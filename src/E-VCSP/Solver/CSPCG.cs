using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.ColumnGenerators;
using Gurobi;
using System.Collections;
using System.Data;

namespace E_VCSP.Solver
{
    enum BlockArcType
    {
        Break,
        LongIdle,
        ShortIdle,
        Travel,
        SignOnOff,
        Invalid,
    }


    internal class BlockArc
    {
        internal Block? FromBlock;
        internal Block? ToBlock;
        internal int BreakTime;
        internal int BruttoNettoTime;
        internal int IdleTime;
        internal int TravelTime;
        internal BlockArcType Type = BlockArcType.Invalid;

        internal int TotalTime => BreakTime + BruttoNettoTime + IdleTime + TravelTime;
    }


    public class CSPCG : Solver
    {
        private GRBModel? model;
        private Instance instance;
        private List<List<BlockArc>> adj = [];
        private List<List<BlockArc?>> adjFull = [];


        private List<CrewDuty> duties = [];
        private Dictionary<string, CrewDuty> varnameDutyMapping = [];
        private Dictionary<BitArray, CrewDuty> coverDutyMapping = new(new Utils.BitArrayComparer());

        public CSPCG(Instance instance)
        {
            this.instance = instance;
            this.duties = [];

            // Generate initial set of vehicle duties
            GenerateInitialDuties();

            // Generate possible transfers between blocks
            GenerateArcs();

            Console.WriteLine($"Ready for CSP: total of {instance.Blocks.Count} blocks to cover");
        }

        private void GenerateInitialDuties()
        {
            // For each block, generate a unit duty
            for (int i = 0; i < instance.Blocks.Count; i++)
            {
                duties.Add(new CrewDuty([new CDEBlock(instance.Blocks[i])]) { Index = i, Type = DutyType.Single });
            }
        }

        private void GenerateArcs()
        {
            adjFull = Enumerable.Range(0, instance.Blocks.Count + 2).Select(x => new List<BlockArc?>()).ToList();
            adj = Enumerable.Range(0, instance.Blocks.Count + 2).Select(x => new List<BlockArc>()).ToList();

            for (int blockIndex1 = 0; blockIndex1 < instance.Blocks.Count; blockIndex1++)
            {
                Block block1 = instance.Blocks[blockIndex1];
                for (int blockIndex2 = 0; blockIndex2 < instance.Blocks.Count; blockIndex2++)
                {
                    Block block2 = instance.Blocks[blockIndex2];

                    // Determine whether or not it is feasible to string to arcs together
                    // If so: what actually happens during this time. 

                    // For now; only allow transfer if already at same location
                    // Based on times, determine whether its idle / break / whatever. 
                    BlockArc? arc = null;

                    if (block1.EndTime <= block2.StartTime && block1.EndLocation == block2.StartLocation)
                    {
                        // Arc might be formed; start with base time layout, check for validity
                        BlockArcType blockArcType = BlockArcType.Invalid;
                        int idleTime = block2.StartTime - block1.EndTime;
                        int breakTime = 0;
                        int bruttoNettoTime = 0;
                        int travelTime = 0;
                        int nettoBreakTime = block1.EndLocation.BreakAllowed
                            ? idleTime - block1.EndLocation.BrutoNetto
                            : 0;

                        if (nettoBreakTime >= Config.CR_MIN_BREAK_TIME && nettoBreakTime <= Config.CR_MAX_BREAK_TIME)
                        {
                            breakTime = idleTime - block1.EndLocation.BrutoNetto;
                            bruttoNettoTime = block1.EndLocation.BrutoNetto;
                            idleTime = 0;
                            blockArcType = BlockArcType.Break;
                        }

                        // Short idle; can happen anywhere (driver remains in bus)
                        else if (Config.CR_MIN_SHORT_IDLE_TIME <= idleTime && idleTime <= Config.CR_MAX_SHORT_IDLE_TIME)
                            blockArcType = BlockArcType.ShortIdle;

                        // Long idle used for split shifts
                        else if (block1.EndLocation.CrewHub && Config.CR_MIN_LONG_IDLE_TIME <= idleTime && idleTime <= Config.CR_MAX_LONG_IDLE_TIME)
                            blockArcType = BlockArcType.LongIdle;

                        if (blockArcType != BlockArcType.Invalid)
                        {
                            arc = new()
                            {
                                FromBlock = block1,
                                ToBlock = block2,
                                IdleTime = idleTime,
                                BreakTime = breakTime,
                                BruttoNettoTime = bruttoNettoTime,
                                TravelTime = travelTime,
                                Type = blockArcType
                            };
                        }
                    }

                    if (arc != null) adj[blockIndex1].Add(arc);
                    adjFull[blockIndex1].Add(arc);
                }
            }

            // Add depot arcs if signon / signoff is allowed
            for (int blockIndex = 0; blockIndex < instance.Blocks.Count; blockIndex++)
            {
                Block block = instance.Blocks[blockIndex];
                BlockArc? start = block.StartLocation.CrewHub ? new BlockArc()
                {
                    ToBlock = block,
                    IdleTime = 0,
                    BreakTime = 0,
                    BruttoNettoTime = 0,
                    TravelTime = block.StartLocation.SignOnTime,
                    Type = BlockArcType.SignOnOff
                } : null;
                if (start != null) adj[^2].Add(start);
                adjFull[^2].Add(start);
                adjFull[blockIndex].Add(null);

                BlockArc? end = block.EndLocation.CrewHub ? new BlockArc()
                {
                    FromBlock = block,
                    IdleTime = 0,
                    BreakTime = 0,
                    BruttoNettoTime = 0,
                    TravelTime = block.StartLocation.SignOffTime,
                    Type = BlockArcType.SignOnOff
                } : null;
                if (end != null)
                    adj[block.Index].Add(end);
                adjFull[block.Index].Add(end);
            }
        }

        /// <summary>
        /// Initialize model for column generaton
        /// </summary>
        /// <param name="ct">Cancellation token</param>
        /// <returns>Model where first <c>n</c> constraints correspond to the <c>n</c> trips, and a list of initial vehicle task vars</returns>
        private (GRBModel model, List<GRBVar> dutyVars) InitModel(CancellationToken ct)
        {
            // Env
            GRBEnv env = new()
            {
                LogToConsole = 1,
                LogFile = Path.Combine(Config.RUN_LOG_FOLDER, "cspcg_gurobi.log")
            };

            // Model
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.CSP_SOLVER_TIMEOUT_SEC;
            model.SetCallback(new CustomGRBCallback());
            ct.Register(() =>
            {
                Console.WriteLine("Cancellation requested during crew scheduling. Terminating Gurobi model...");
                model?.Terminate();
            });

            List<GRBVar> dutyVars = [];
            for (int i = 0; i < duties.Count; i++)
            {
                string name = $"cd_{i}";
                GRBVar v = model.AddVar(0, GRB.INFINITY, duties[i].Cost + 1_000, GRB.CONTINUOUS, name);

                // Bookkeeping to find variable based on name / cover easily
                dutyVars.Add(v);
                varnameDutyMapping[name] = duties[i];
                coverDutyMapping.Add(duties[i].ToBitArray(instance.Blocks.Count), duties[i]);
            }

            // Add cover constraint for each of the trips
            // Note: index of constraint corresponds directly to index of trip 
            foreach (Block b in instance.Blocks)
            {
                GRBLinExpr expr = new();
                for (int i = 0; i < duties.Count; i++)
                {
                    if (duties[i].Covers.Contains(b.Index)) expr.AddTerm(1, dutyVars[i]);
                }

                // Switch between set partition and cover
                char sense = Config.CSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                model.AddConstr(expr, sense, 1, "cover_" + b.Index);
            }

            // Shift type/time limit constraints
            GRBLinExpr noExcessiveLength = new(); // max 15% > 8.5h
            GRBLinExpr limitedAverageLength = new(); // avg 8 hours
            GRBLinExpr maxBroken = new(); // max 30% broken
            GRBLinExpr maxBetween = new(); // max 10% between
            GRBLinExpr maxSingle = new(); // Use singles at a cost

            GRBVar noExcessiveLengthSlack = model.AddVar(0, GRB.INFINITY, 10, GRB.CONTINUOUS, "noExcessiveLengthSlack");
            GRBVar limitedAverageLengthSlack = model.AddVar(0, GRB.INFINITY, 10, GRB.CONTINUOUS, "limitedAverageLengthSlack");
            GRBVar maxBrokenSlack = model.AddVar(0, GRB.INFINITY, 10, GRB.CONTINUOUS, "maxBrokenSlack");
            GRBVar maxBetweenSlack = model.AddVar(0, GRB.INFINITY, 10, GRB.CONTINUOUS, "maxBetweenSlack");

            for (int i = 0; i < dutyVars.Count; i++)
            {
                GRBVar v = dutyVars[i];
                CrewDuty duty = duties[i];

                int duration = duty.Elements[^1].EndTime - duty.Elements[0].StartTime;
                noExcessiveLength += Config.CR_MAX_OVER_LONG_SHIFT * v - (duration > Config.CR_LONG_SHIFT_LENGTH ? v : 0);
                limitedAverageLength += v * (duration / (double)Config.CR_TARGET_SHIFT_LENGTH - 1);
                maxBroken += v * Config.CR_MAX_BROKEN_SHIFTS - (duty.Type == DutyType.Broken ? v : 0);
                maxBetween += v * Config.CR_MAX_BETWEEN_SHIFTS - (duty.Type == DutyType.Between ? v : 0);
            }


            model.AddConstr(noExcessiveLength + noExcessiveLengthSlack >= 0, "no_excessive_length");
            model.AddConstr(limitedAverageLength - limitedAverageLengthSlack <= 0, "limited_average_length");
            model.AddConstr(maxBroken + maxBrokenSlack >= 0, "max_broken");
            model.AddConstr(maxBetween + maxBetweenSlack >= 0, "max_between");

            this.model = model;
            return (model, dutyVars);
        }

        private List<CrewDuty> getSelectedDuties()
        {
            if (model == null) throw new InvalidOperationException("Cannot generate solution graph without model instance");

            List<CrewDuty> duties = [];
            int[] covered = new int[instance.Blocks.Count];
            foreach (GRBVar v in model.GetVars())
            {
                if (v.VarName.StartsWith("cd_") && v.X == 1)
                {
                    CrewDuty dvt = varnameDutyMapping[v.VarName];
                    duties.Add(dvt);
                    foreach (int i in dvt.Covers) covered[i]++;
                }
            }

            int coveredTotal = 0;
            for (int i = 0; i < covered.Length; i++)
            {
                int val = covered[i];
                if (val >= 1) coveredTotal++;
                if (val >= 2) Console.WriteLine($"(!) Block {instance.Blocks[i]} covered {val} times");
            }

            Console.WriteLine($"Covered {coveredTotal}/{instance.Blocks.Count} blocks using {duties.Count} duties");
            if (coveredTotal < instance.Blocks.Count)
                Console.WriteLine("(!) Not all blocks covered");

            return duties;
        }

        public override bool Solve(CancellationToken cancellationToken)
        {
            (var model, var dutyVars) = InitModel(cancellationToken);
            model.Optimize();

            // Tracking generated columns
            int maxColumns = Config.CSP_INSTANCES_PER_IT * Config.CSP_MAX_COL_GEN_ITS,
                lastReportedPercent = 0,    // Percentage of total reporting
                currIts = 1,                // Number of CG / solution rounds had
                totalGenerated = 0,         // Total number of columns generated
                lbGenerated = 0,            // Generated with labeling
                seqWithoutRC = 0,           // Number of sequential columns without reduced cost found
                totWithoutRC = 0,           // Total columns generated with no RC
                notFound = 0;               // Number of columns that could not be generated

            // Multithreaded shortestpath searching
            List<List<CrewColumnGen>> instances = [
                [.. Enumerable.Range(0, Config.CSP_INSTANCES_PER_IT).Select(_ => new CSPLabeling(instance.Blocks, model, adj, adjFull))], // Labeling
                [.. Enumerable.Range(0, Config.CSP_INSTANCES_PER_IT).Select(_ => new CSPLSGlobal(instance.Blocks, model, adj, adjFull))], // Labeling
            ];
            List<double> operationChances = [Config.CSP_LABELING_WEIGHT, Config.CSP_LS_GLOBAL_WEIGHT];
            List<double> sums = [operationChances[0]];
            for (int i = 1; i < operationChances.Count; i++) sums.Add(sums[i - 1] + operationChances[i]);

            Console.WriteLine("CSP Column generation started");
            Console.WriteLine("%\tT\tLB\tNF\tDN\tDO\tWRC\tMV");

            Random rnd = new();

            // Process a collection of generated duties
            void processDuties(List<(double reducedCost, CrewDuty cd)> dutySet)
            {
                if (dutySet.Count == 0)
                {
                    notFound++;
                    return;
                }

                foreach (var task in dutySet)
                {
                    (double reducedCost, CrewDuty newDuty) = ((double, CrewDuty))task;

                    // Check if task is already in model 
                    BitArray ba = newDuty.ToBitArray(instance.Trips.Count);
                    bool coverExists = coverDutyMapping.ContainsKey(ba);

                    // Add column to model 
                    if (reducedCost < 0)
                    {
                        // Reset non-reduced costs iterations
                        int index = duties.Count;
                        string name = $"cd_{index}";
                        duties.Add(newDuty);
                        newDuty.Index = index;

                        // Create new column to add to model
                        var modelConstrs = model.GetConstrs();
                        GRBConstr[] constrs = [.. modelConstrs.Where(
                            (_, i) => newDuty.Covers.Contains(i) || i >= instance.Blocks.Count  // Covers block || one of the shift type.length constraints
                        )];
                        GRBColumn col = new();
                        col.AddTerms([.. constrs.Select(c => {
                            if (c.ConstrName.StartsWith("cover_")) return 1.0;
                            else if (c.ConstrName == "no_excessive_length") return newDuty.Duration > Config.CR_LONG_SHIFT_LENGTH ? Config.CR_MAX_OVER_LONG_SHIFT - 1 : Config.CR_MAX_OVER_LONG_SHIFT;
                            else if (c.ConstrName == "limited_average_length") return (newDuty.Duration / (double)Config.CR_TARGET_SHIFT_LENGTH - 1);
                            else if (c.ConstrName == "max_broken") return newDuty.Type == DutyType.Broken ? Config.CR_MAX_BROKEN_SHIFTS - 1 : Config.CR_MAX_BROKEN_SHIFTS;
                            else if (c.ConstrName == "max_between") return newDuty.Type == DutyType.Between ? Config.CR_MAX_BETWEEN_SHIFTS - 1 : Config.CR_MAX_BETWEEN_SHIFTS;
                            else if (c.ConstrName == "max_single") return 0;
                            else throw new InvalidOperationException($"Constraint {c.ConstrName} not handled when adding new column");
                        })], constrs);

                        // Add column to model
                        dutyVars.Add(model.AddVar(0, GRB.INFINITY, newDuty.Cost, GRB.CONTINUOUS, col, name));
                        varnameDutyMapping[name] = duties[^1];
                        coverDutyMapping[ba] = duties[^1];
                    }
                    else
                    {
                        seqWithoutRC++;
                        totWithoutRC++;
                    }
                }
            }

            // Continue until max number of columns is found, model isn't feasible during solve or break
            // due to RC constraint. 
            while (currIts < Config.CSP_MAX_COL_GEN_ITS && model.Status != GRB.Status.INFEASIBLE && !cancellationToken.IsCancellationRequested)
            {
                // Display progress
                int percent = (int)((totalGenerated / (double)maxColumns) * 100);
                if (percent >= lastReportedPercent + 10)
                {
                    lastReportedPercent = percent - (percent % 10);
                    Console.WriteLine($"{lastReportedPercent}%\t{totalGenerated}\t{lbGenerated}\t{notFound}\t{totWithoutRC}\t{model.ObjVal}");
                }

                // Select solution strategy, generate new duties
                List<(double, CrewDuty)>[] generatedDuties = new List<(double, CrewDuty)>[Config.CSP_INSTANCES_PER_IT];
                double r = rnd.NextDouble() * sums[^1];
                int selectedMethodÍndex = sums.FindIndex(x => r <= x);
                List<CrewColumnGen> selectedMethod = instances[selectedMethodÍndex];
                Parallel.For(0, Config.CSP_INSTANCES_PER_IT, (i) => generatedDuties[i] = selectedMethod[i].GenerateDuties());

                // Update generated totals
                totalGenerated += generatedDuties.Length;
                int colsGenerated = generatedDuties.Sum(t => t.Count);
                switch (selectedMethodÍndex)
                {
                    case 0: lbGenerated += colsGenerated; break;
                    default: throw new InvalidOperationException("You forgot to add a case");
                }

                // Add new duty columns to model, keep track of totals 
                foreach (var dutySet in generatedDuties) processDuties(dutySet);

                // Resolve model
                model.Update();
                model.Set(GRB.IntParam.Presolve, 0);
                model.Optimize();
                currIts++;

                // Stop model solving if no improvements are found
                if (seqWithoutRC >= Config.CSP_OPT_IT_THRESHOLD)
                {
                    Console.WriteLine($"Stopped due to RC > 0 for {Config.CSP_OPT_IT_THRESHOLD} consecutive duties");
                    break;
                }
            }

            if (model.Status == GRB.Status.INFEASIBLE || model.Status == GRB.Status.INTERRUPTED)
            {
                Console.WriteLine("Model infeasible / canceled");
                model.ComputeIIS();
                model.Write("infeasible_CSPCG.ilp");
                return false;
            }

            // Info dump
            Console.WriteLine($"100%\t{totalGenerated}\t{lbGenerated}\t{notFound}\t{totWithoutRC}\t{model.ObjVal}");
            Console.WriteLine($"Value of relaxation: {model.ObjVal}");
            Console.WriteLine($"Total generation attempts: ${totalGenerated}");
            Console.WriteLine($"{totWithoutRC} columns were not added due to positive reduced costs.");
            Console.WriteLine($"Solving non-relaxed model with total of {duties.Count} columns");

            // Make model binary again
            foreach (GRBVar var in dutyVars)
            {
                if (var.VarName.StartsWith("cd_"))
                    var.Set(GRB.CharAttr.VType, GRB.BINARY);
            }
            model.Update();

            bool configState = Config.CONSOLE_GUROBI;
            Config.CONSOLE_GUROBI = true; // Force enable console at end as this solve takes a long time
            model.Optimize();

            bool succes = !(model.Status == GRB.Status.INFEASIBLE || model.Status == GRB.Status.INTERRUPTED);

            if (!succes)
            {
                Console.WriteLine("Model infeasible / canceled");
                model.ComputeIIS();
                model.Write("infeasible_CSPCG.ilp");
            }
            else Console.WriteLine($"Costs: {model.ObjVal}");

            Config.CONSOLE_GUROBI = configState;
            instance.SelectedDuties = getSelectedDuties();
            return succes;
        }
    }
}

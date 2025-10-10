using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using System.Collections;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace E_VCSP.Solver.SolutionState {
    public class VehicleSolutionStateDump {
        [JsonInclude]
        public required string path;
        // Only include selected tasks
        [JsonInclude]
        public List<VehicleTask> selectedTasks = [];
        [JsonInclude]
        public required VehicleType vehicleType;
    }

    public abstract class EVSPNode {
        public required int Index;
    }
    public class TripNode : EVSPNode {
        public required Trip Trip;
    }
    public class DepotNode : EVSPNode {
        public required Location Depot;
    }

    public class VSPArc {
        public required int StartTime;
        public required int EndTime;
        public required EVSPNode From;
        public required EVSPNode To;
        public required DeadheadTemplate DeadheadTemplate;
    }

    public class VehicleSolutionState {
        public Instance Instance;

        public VehicleType VehicleType;

        public List<VehicleTask> SelectedTasks = [];
        public List<VehicleTask> Tasks = [];
        public Dictionary<BitArray, VehicleTask> CoverTaskMapping = new(new Utils.BitArrayComparer());
        public Dictionary<string, VehicleTask> VarnameTaskMapping = [];

        public List<EVSPNode> Nodes = [];
        public List<List<VSPArc?>> AdjFull = [];
        public List<List<VSPArc>> Adj = [];
        public List<List<DeadheadTemplate?>> LocationDHT = [];

        public Location Depot;
        public int StartTime = int.MaxValue;
        public int EndTime = int.MinValue;

        #region Initialization
        public VehicleSolutionState(Instance instance, VehicleType vehicleType) {
            this.Instance = instance;
            this.VehicleType = vehicleType;

            Depot = instance.Locations.Find(x => x.IsDepot) ?? throw new InvalidDataException("No depot found");
            Reset();
        }

        public void Dump() {
            File.WriteAllText(Constants.RUN_LOG_FOLDER + "vss-result.json", JsonSerializer.Serialize(new VehicleSolutionStateDump() {
                selectedTasks = SelectedTasks,
                vehicleType = VehicleType,
                path = Instance.Path,
            }));
        }

        public void LoadFromDump(VehicleSolutionStateDump dump) {
            VehicleType = Instance.VehicleTypes.Find(x => dump.vehicleType.Id == x.Id)!;

            dump.selectedTasks = dump.selectedTasks.Select(t => {
                t.vehicleType = VehicleType;
                t.Elements = t.Elements.Select(e => {
                    e.EndLocation = Instance.Locations.Find(l => l.Id == e.EndLocation!.Id);
                    e.StartLocation = Instance.Locations.Find(l => l.Id == e.StartLocation!.Id);
                    if (e is VETrip et) {
                        var tripMatch = Instance.Trips.Find(t => t.Id == et.Trip.Id);
                        if (tripMatch == null)
                            throw new InvalidDataException();
                        et.Trip = tripMatch;
                    }
                    if (e is VEDeadhead ed) {
                        var dhtMatch = Instance.ExtendedTemplates.Find(dht => dht.Id == ed.DeadheadTemplate.Id);
                        if (dhtMatch == null)
                            throw new InvalidDataException();
                        ed.DeadheadTemplate = dhtMatch;
                    }
                    return e;
                }).ToList();
                return t;
            }).ToList();

            SelectedTasks = dump.selectedTasks;
            Tasks = dump.selectedTasks;
            PrintCostBreakdown();
        }

        public void Reset(bool withTasks = true) {
            // Reset default vals
            SelectedTasks = [];
            Tasks = [];
            CoverTaskMapping = new(new Utils.BitArrayComparer());
            VarnameTaskMapping = [];
            StartTime = int.MaxValue;
            EndTime = int.MinValue;
            Nodes = [];
            AdjFull = [];
            Adj = [];
            LocationDHT = [];

            // Initialize depot
            Depot = Instance.Locations.Find(x => x.IsDepot) ?? throw new InvalidDataException("No depot found");

            // Initialize rest
            generateLocationDHT();
            generateGraph();
            if (withTasks) generateInitialTasks();

            // Min / max times
            foreach (Trip t in Instance.Trips) {
                // Depot to trip
                int minDepTime = t.StartTime - AdjFull[Instance.DepotStartIndex][t.Index]!.DeadheadTemplate.Duration;
                StartTime = Math.Min(minDepTime, StartTime);

                // Trip to depot
                int maxArrTime = t.EndTime + AdjFull[t.Index][Instance.DepotEndIndex]!.DeadheadTemplate.Duration;
                EndTime = Math.Max(maxArrTime, EndTime);
            }
        }
        #endregion

        #region Initialize Graph and Tasks
        private void generateGraph() {
            // Transform trips into nodes, add depot start/end
            // Depot start = ^2, depot end = ^1
            for (int i = 0; i < Instance.Trips.Count; i++) {
                Nodes.Add(new TripNode() { Index = i, Trip = Instance.Trips[i] });
            }

            Nodes.Add(new DepotNode() { Index = Nodes.Count, Depot = Depot }); // Start depot
            Nodes.Add(new DepotNode() { Index = Nodes.Count, Depot = Depot }); // End depot


            // Initialize adjacency lists
            for (int i = 0; i < Nodes.Count; i++) {
                AdjFull.Add([]);
                Adj.Add([]);
                for (int j = 0; j < Nodes.Count; j++) {
                    AdjFull[i].Add(null);
                }
            }

            // depot start -> trip arcs
            for (int i = 0; i < Nodes.Count - 2; i++) {
                TripNode tn = (TripNode)Nodes[i];
                DeadheadTemplate? dht = LocationDHT[Depot.Index][tn.Trip.StartLocation.Index] ?? throw new InvalidDataException("No travel possible from depot to trip");
                VSPArc arc = new VSPArc() {
                    StartTime = tn.Trip.StartTime - dht.Duration,
                    EndTime = tn.Trip.StartTime,
                    From = Nodes[^2],
                    To = tn,
                    DeadheadTemplate = dht
                };
                AdjFull[^2][i] = arc;
                Adj[^2].Add(arc);
            }
            // trip -> depot end arcs
            for (int i = 0; i < Nodes.Count - 2; i++) {
                TripNode tn = (TripNode)Nodes[i];
                DeadheadTemplate? dht = LocationDHT[tn.Trip.EndLocation.Index][Depot.Index] ?? throw new InvalidDataException("No travel possible from trip to depot");
                VSPArc arc = new VSPArc() {
                    StartTime = tn.Trip.EndTime,
                    EndTime = tn.Trip.EndTime + dht.Duration,
                    From = tn,
                    To = Nodes[^1],
                    DeadheadTemplate = dht
                };
                AdjFull[i][^1] = arc;
                Adj[i].Add(arc);
            }
            // depot -> depot arc
            {
                DeadheadTemplate? dht = LocationDHT[Depot.Index][Depot.Index] ?? throw new InvalidDataException("No travel possible from depot to depot");
                VSPArc arc = new VSPArc() {
                    StartTime = 0,
                    EndTime = 0,
                    From = Nodes[^2],
                    To = Nodes[^1],
                    DeadheadTemplate = dht
                };
                AdjFull[^2][^1] = arc;
                Adj[^2].Add(arc);
            }

            int totalSimplified = 0;

            // Trip to trip arcs
            for (int i = 0; i < Nodes.Count - 2; i++) {
                TripNode tn1 = (TripNode)Nodes[i];

                for (int j = 0; j < Nodes.Count - 2; j++) {
                    if (i == j) continue;

                    TripNode tn2 = (TripNode)Nodes[j];
                    DeadheadTemplate? dht = LocationDHT[tn1.Trip.EndLocation.Index][tn2.Trip.StartLocation.Index];
                    if (dht == null) continue; // not a possible drive
                    if (tn1.Trip.EndTime + dht.Duration > tn2.Trip.StartTime) continue; // Deadhead not time feasible

                    if (dht.FreqChangeOnly) {
                        if (!Config.VSP_ALLOW_UNKNOWN_DHTS) continue;

                        // Only allowed if at least one of the trips has a frequency change
                        bool tn1FreqChange =
                            tn1.Trip.FrequencyChange == FrequencyChange.EndOfTrip
                            || tn1.Trip.FrequencyChange == FrequencyChange.SingleTrip;
                        bool tn2FreqChange =
                            tn2.Trip.FrequencyChange == FrequencyChange.StartOfTrip
                            || tn2.Trip.FrequencyChange == FrequencyChange.SingleTrip;
                        if (!tn1FreqChange && !tn2FreqChange) continue;
                    }

                    VSPArc arc = new VSPArc() {
                        StartTime = tn1.Trip.EndTime,
                        EndTime = tn2.Trip.StartTime,
                        To = tn2,
                        From = tn1,
                        DeadheadTemplate = dht
                    };

                    AdjFull[i][j] = arc;
                    Adj[i].Add(arc);
                }
            }

            double avgOutgoingBeforeSimplify = Adj.Where((x, i) => i < Instance.Trips.Count).Sum(x => x.Count) / ((double)Adj.Count - 2);

            // Preprocessing: filter by min length, if one is found with low time only use that 
            for (int i = 0; i < Nodes.Count - 2; i++) {
                TripNode tn1 = (TripNode)Nodes[i];
                HashSet<int> directArcIndexes = [];
                HashSet<int> allowsForChargeIndexes = [];

                for (int dai = 0; dai < Adj[i].Count; dai++) {
                    VSPArc arc = Adj[i][dai];
                    if (arc.To.Index < Instance.Trips.Count) {
                        Trip t2 = ((TripNode)arc.To).Trip;

                        int timeDiff = t2.StartTime - tn1.Trip.EndTime;
                        if (timeDiff <= Config.VSP_PRE_DIRECT_TIME) {
                            directArcIndexes.Add(t2.Index);
                        }

                        int idleTime = timeDiff - arc.DeadheadTemplate.Duration;
                        if (tn1.Trip.EndLocation.CanCharge && idleTime > Constants.MIN_CHARGE_TIME) {
                            // Charging can be done at the start
                            var chargeRes = tn1.Trip.EndLocation.ChargingCurves[VehicleType.Index].MaxChargeGained(VehicleType.MinSoC, idleTime);
                            if (chargeRes.TimeUsed == idleTime) {
                                // Arc may be useful during charging
                                allowsForChargeIndexes.Add(t2.Index);
                            }
                        }
                    }
                }

                HashSet<int> remainingIndexes = directArcIndexes.Union(allowsForChargeIndexes).ToHashSet();

                if (remainingIndexes.Count > 0) {
                    totalSimplified++;
                    Adj[i] = Adj[i].Where((x, i) => x.To.Index >= Instance.Trips.Count || remainingIndexes.Contains(x.To.Index)).ToList();
                    AdjFull[i] = AdjFull[i].Select((x, i) => x == null || x.To.Index >= Instance.Trips.Count || remainingIndexes.Contains(x.To.Index) ? x : null).ToList();
                }
            }

            double avgOutgoingAfterSimplify = Adj.Where((x, i) => i < Instance.Trips.Count).Sum(x => x.Count) / ((double)Adj.Count - 2);

            Console.WriteLine($"Simplified the arcs of {totalSimplified} trips");
            Console.WriteLine($"Avg outgoing arcs before/after simplification: {avgOutgoingBeforeSimplify}/{avgOutgoingAfterSimplify}");
        }

        private void generateLocationDHT() {
            foreach (Location l1 in Instance.Locations) {
                LocationDHT.Add([]);
                foreach (Location l2 in Instance.Locations) {
                    DeadheadTemplate? dht = Instance.DeadheadTemplates.Find((x) => x.StartLocation == l1 && x.EndLocation == l2);
                    LocationDHT[l1.Index].Add(dht);
                }
            }
        }

        private void generateInitialTasks() {
            for (int i = 0; i < Instance.Trips.Count; i++) {
                Trip t = Instance.Trips[i];
                DeadheadTemplate dhTo = AdjFull[Instance.DepotStartIndex][i]?.DeadheadTemplate ?? throw new InvalidDataException("No arc from depot to trip");
                DeadheadTemplate dhFrom = AdjFull[i][Instance.DepotEndIndex]?.DeadheadTemplate ?? throw new InvalidDataException("No arc from trip to depot");

                double currSoC = VehicleType.StartSoC;
                VEDeadhead toTrip = new(dhTo, Instance.Trips[i].StartTime - dhTo.Duration, Instance.Trips[i].StartTime, VehicleType) {
                    StartSoCInTask = currSoC,
                    EndSoCInTask = currSoC - dhTo.Distance * VehicleType.DriveUsage
                };
                currSoC -= dhTo.Distance * VehicleType.DriveUsage;
                VETrip trip = new(t, VehicleType) {
                    StartSoCInTask = currSoC,
                    EndSoCInTask = currSoC - t.Distance * VehicleType.DriveUsage
                };
                currSoC -= t.Distance * VehicleType.DriveUsage;
                VEDeadhead fromTrip = new(dhFrom, Instance.Trips[i].EndTime, Instance.Trips[i].EndTime + dhFrom.Duration, VehicleType) {
                    StartSoCInTask = currSoC,
                    EndSoCInTask = currSoC - dhFrom.Distance * VehicleType.DriveUsage
                };

                VehicleTask vehicleTask = new([toTrip, trip, fromTrip]) {
                    vehicleType = VehicleType,
                    Index = i,
                    Source = "Base cover",
                };

                Tasks.Add(vehicleTask);
            }
        }
        #endregion

        public void RemoveOvercoverageFromTasks(List<VehicleTask> vehicleTasks) {
            // Determine overcoverage
            List<List<int>> coveredBy = Enumerable.Range(0, Instance.Trips.Count).Select(x => new List<int>()).ToList();
            for (int i = 0; i < vehicleTasks.Count; i++) {
                foreach (int j in vehicleTasks[i].TripIndexCover) {
                    coveredBy[j].Add(i);
                }
            }

            // Two phase approach: 
            // Phase 1: replace all trips with dummy idle blocks with current from/to and currSoCs
            // Phase 2: combine adjacent blocks into 1 deadhead / idle combo. 

            // Phase 1: dummy idle blocks
            for (int i = 0; i < coveredBy.Count; i++) {
                List<int> coverList = coveredBy[i];
                if (coverList.Count == 1) continue;

                // No consideration for which one is target; just use first.
                for (int j = 1; j < coverList.Count; j++) {
                    VehicleTask vt = vehicleTasks[coverList[j]];
                    int veIndex = vt.Elements.FindIndex(x => x.Type == VEType.Trip && ((VETrip)x).Trip.Index == i);
                    VehicleElement ve = vt.Elements[veIndex];
                    Console.WriteLine($"Removing {ve} from vt {coverList[j]}");

                    // Check next / prev to see if we can combine;
                    VehicleElement? prevInteresting = null;
                    int prevInterestingIndex = veIndex - 1;
                    while (prevInterestingIndex >= 0 && prevInteresting == null) {
                        List<VEType> interestingTypes = [VEType.Deadhead, VEType.Charge, VEType.Trip];
                        if (interestingTypes.Contains(vt.Elements[prevInterestingIndex].Type)
                            || (vt.Elements[prevInterestingIndex].Type == VEType.Idle && ((VEIdle)vt.Elements[prevInterestingIndex]).Postprocessed)
                        ) {
                            prevInteresting = vt.Elements[prevInterestingIndex];
                            break;
                        }
                        else prevInterestingIndex--;
                    }

                    VehicleElement? nextInteresting = null;
                    int nextInterestingIndex = veIndex + 1;
                    while (nextInterestingIndex < vt.Elements.Count && nextInteresting == null) {
                        List<VEType> interestingTypes = [VEType.Deadhead, VEType.Charge, VEType.Trip];
                        if (interestingTypes.Contains(vt.Elements[nextInterestingIndex].Type)
                            || (vt.Elements[nextInterestingIndex].Type == VEType.Idle && ((VEIdle)vt.Elements[nextInterestingIndex]).Postprocessed)
                        ) {
                            nextInteresting = vt.Elements[nextInterestingIndex];
                            break;
                        }
                        else nextInterestingIndex++;
                    }

                    List<VehicleElement> newElements = [];


                    Location from = prevInteresting?.EndLocation ?? ((VETrip)ve).Trip.StartLocation;
                    Location to = nextInteresting?.StartLocation ?? ((VETrip)ve).Trip.EndLocation;
                    int startIndex = prevInterestingIndex + 1;
                    int endIndex = nextInterestingIndex - 1;

                    // We want to glue the two interesting parts together; if either end is a deadhead, more can be compressed.
                    if (prevInteresting != null && (prevInteresting.Type == VEType.Deadhead || ((prevInteresting.Type == VEType.Idle && ((VEIdle)prevInteresting).Postprocessed)))) {
                        from = prevInteresting.StartLocation!;
                        startIndex = prevInterestingIndex;
                    }
                    if (nextInteresting != null && (nextInteresting.Type == VEType.Deadhead || ((nextInteresting.Type == VEType.Idle && ((VEIdle)nextInteresting).Postprocessed)))) {
                        to = nextInteresting.EndLocation!;
                        endIndex = nextInterestingIndex;
                    }

                    // Connect t
                    double startSoC = vt.Elements[startIndex].StartSoCInTask;
                    double endSoC = vt.Elements[endIndex].EndSoCInTask;
                    int startTime = vt.Elements[startIndex].StartTime;
                    int endTime = vt.Elements[endIndex].EndTime;

                    // 
                    var dummyIdle = new VEIdle(to, startTime, endTime) {
                        StartLocation = from,
                        EndLocation = to,
                        StartSoCInTask = startSoC,
                        EndSoCInTask = endSoC,
                        Postprocessed = true,
                    };

                    for (int x = startIndex; x <= endIndex; x++) {
                        if (vt.Elements[x].Type == VEType.Idle || vt.Elements[x].Type == VEType.Deadhead || (vt.Elements[x].Type == VEType.Trip && ((VETrip)vt.Elements[x]).Trip.Index == i)) continue;
                        throw new InvalidOperationException("verkeerde ding weg aan t gooien");
                    }

                    vt.Elements.RemoveRange(startIndex, endIndex - startIndex + 1);
                    vt.Elements.InsertRange(startIndex, [dummyIdle]);
                }
            }

            // Phase 2: combining idle blocks, adding deadheads
            for (int i = 0; i < vehicleTasks.Count; i++) {
                VehicleTask vt = vehicleTasks[i];
                List<(int index, VEIdle ve)> idles = [];

                void replace() {
                    Location from = idles[0].ve.StartLocation!;
                    Location to = idles[^1].ve.EndLocation!;
                    DeadheadTemplate dht = Instance.ExtendedTemplates.Find(x => x.StartLocation == from && x.EndLocation == to)!;
                    double startSoC = idles[0].ve.StartSoCInTask;
                    double endSoC = idles[^1].ve.EndSoCInTask;
                    int startTime = idles[0].ve.StartTime;
                    int endTime = idles[^1].ve.EndTime;
                    int idleTime = endTime - startTime - dht.Duration;

                    var newDeadhead = new VEDeadhead(dht, startTime, startTime + dht.Duration, VehicleType) {
                        StartSoCInTask = startSoC,
                        EndSoCInTask = endSoC,
                        Postprocessed = true,
                    };
                    var newIdle = new VEIdle(to, startTime + dht.Duration, endTime) {
                        StartSoCInTask = endSoC,
                        EndSoCInTask = endSoC,
                        Postprocessed = true,
                    };
                    vt.Elements.RemoveRange(idles[0].index, idles.Count);
                    vt.Elements.InsertRange(idles[0].index, [newDeadhead, newIdle]);

                    idles.Clear();
                }

                for (int j = 0; j < vt.Elements.Count; j++) {
                    VehicleElement ve = vt.Elements[j];
                    if (ve.Type == VEType.Idle && ((VEIdle)ve).Postprocessed) {
                        idles.Add((j, (VEIdle)ve));
                    }
                    else if (idles.Count > 0) {
                        replace();
                        j = j + 2 - idles.Count;
                    }
                }

                if (idles.Count > 0) replace();
            }

            // Reset calculated costs as they may have changed
            foreach (var vt in vehicleTasks) vt.RecalcTripCovers();
        }

        public double Costs() {
            return SelectedTasks.Sum(x => x.Cost);
        }

        public void PrintCostBreakdown(int slack = 0) {
            double overallCost = SelectedTasks.Sum(x => x.Cost);
            double pulloutCost = SelectedTasks.Count * Config.VH_PULLOUT_COST;
            double tripDrivingCost = SelectedTasks.Sum(x => x.Elements.Sum(y => y.Type == VEType.Trip ? y.Cost : 0));
            double tripKWHCost = SelectedTasks.Sum(x => x.Elements.Sum(y => y.Type == VEType.Trip ? Math.Max(0, y.StartSoCInTask - y.EndSoCInTask) / 100 * VehicleType.Capacity * Constants.KWH_COST : 0));
            double dhDrivingCost = SelectedTasks.Sum(x => x.Elements.Sum(y => y.Type == VEType.Deadhead ? y.Cost : 0));
            double dhKWHCost = SelectedTasks.Sum(x => x.Elements.Sum(y => y.Type == VEType.Deadhead ? Math.Max(0, y.StartSoCInTask - y.EndSoCInTask) / 100 * VehicleType.Capacity * Constants.KWH_COST : 0));
            double idleKWHCost = SelectedTasks.Sum(x => x.Elements.Sum(y => y.Type == VEType.Idle ? Math.Max(0, y.StartSoCInTask - y.EndSoCInTask) / 100 * VehicleType.Capacity * Constants.KWH_COST : 0)); ;
            double batteryDegCost = overallCost - pulloutCost - tripDrivingCost - tripKWHCost - dhDrivingCost - dhKWHCost - idleKWHCost;

            string breakdown =
            $"""
            Overall vehicle costs: {SelectedTasks.Sum(x => x.Cost) + slack * Config.VH_OVER_MAX_COST}
            Cost breakdown:
            Tasks: {overallCost}
                Vehicle pullout costs: {pulloutCost}
                Driving: 
                    Trip: {tripDrivingCost}
                    Deadhead: {dhDrivingCost}
                KWh usage:
                    Trip: {tripKWHCost}
                    Deadhead: {dhKWHCost}
                    Idle: {idleKWHCost}
                Battery degredation: {batteryDegCost}
            Slack penalty: {slack * Config.VH_OVER_MAX_COST}
            """;

            Console.WriteLine(breakdown);
        }
    }
}

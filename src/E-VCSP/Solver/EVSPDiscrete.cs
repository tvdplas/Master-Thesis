using E_VCSP.Objects;
using Gurobi;

namespace E_VCSP.Solver
{


    internal class EVSPDiscrete
    {
        private abstract class DeadheadPoint
        {
            internal int Id;
        }
        private class DiscreteTrip : DeadheadPoint
        {
            internal required Trip Trip;
            internal int StartingSoC;
        }
        private class DiscreteDepot : DeadheadPoint
        {
            internal required Location Location;
        }


        internal Instance Instance;
        internal int DiscreteFactor = 10; // Discretize the SoC into 10 

        private double floorToDiscreteValue(double inp) => Math.Floor(inp / DiscreteFactor) * DiscreteFactor;

        public EVSPDiscrete(Instance instance)
        {
            Instance = instance;
        }

        public void Solve()
        {
            GRBEnv env = new();
            GRBModel model = new(env);

            // TODO: misschien iets beter doen dan gewoon de eerste nemen 
            VehicleType vh = Instance.VehicleTypes[0];

            // Create all discrete trips
            List<List<DiscreteTrip>> discreteTrips = new(Instance.Trips.Count);
            for (int i = 0; i < Instance.Trips.Count; i++)
            {
                discreteTrips.Add(new(DiscreteFactor));
                for (int j = 0; j <= DiscreteFactor; j++)
                {
                    discreteTrips[i].Add(new DiscreteTrip()
                    {
                        Trip = Instance.Trips[i],
                        StartingSoC = j * 100 / DiscreteFactor
                    });
                }
            }

            // Add feasible deadheads
            List<(DeadheadPoint, DeadheadPoint, double)> feasibleDeadheads = new();
            foreach ((var dl1, var dl2) in discreteTrips.SelectMany(x => discreteTrips.Select(y => (x, y))))
            {
                foreach ((DiscreteTrip dt1, DiscreteTrip dt2) in dl1.SelectMany(x => dl2.Select(y => (x, y))))
                {
                    // Check each pair of discrete nodes
                    if (dt1.Trip.EndTime > dt2.Trip.StartTime) continue;

                    // Early return when the initial trip can't even be driven at the starting SoC.
                    double SoCusedInDT1 = dt1.Trip.Distance * vh.DriveUsage;
                    if (SoCusedInDT1 > dt1.StartingSoC) continue;

                    // Two possibilities: direct drive from dt1 to dt2 or drive via charging station. Both also need to be time-feasible
                    // First: check if direct drive is timefeasible
                    // If it's not, trips via a charging station aren't feasible either (or deadhead times are incorrect, but we assume that they are consistent).

                    // TODO: use deadheads instead of deadhead templates
                    DeadheadTemplate? dht_direct = Instance.DeadheadTemplates.Find(dht => dht.From == dt1.Trip.To && dht.To == dt2.Trip.From);
                    if (dht_direct != null && dt1.Trip.EndTime + dht_direct.Duration <= dt2.Trip.StartTime)
                    {
                        // Check if charge is compatible between the two trips
                        double SoC = dt1.StartingSoC - SoCusedInDT1;
                        SoC -= dht_direct.Distance * vh.DriveUsage;
                        if (SoC > 0)
                        {
                            // May be compatible; Note that SoC < 0 may still be possible via indirect trip, therefore no early termination. 
                            SoC = floorToDiscreteValue(SoC);
                            if (SoC == dt2.StartingSoC)
                            {
                                // Determine costs of deadhead; only driving time is used
                                // TODO: idle wordt helemaal niet meegenomen
                                double drivingCost = dht_direct.Distance * Config.KM_COST;
                                feasibleDeadheads.Add((dt1, dt2, drivingCost));
                                continue;
                            }
                        }
                    }

                    // Iterate over all charging locations, see whether we can find a valid match in here.
                    // TODO: misschien handig om over alles heen te itereren / duplicate deadheads to te voegen 
                    // om te checken of het goedkoper kan dmv idle time/charging
                    for (int i = 0; i < Instance.ChargingLocations.Count; i++)
                    {
                        Location chargeLoc = Instance.ChargingLocations[i];
                        // Check if detour via charging location is time-feasible.
                        DeadheadTemplate? dht_toCharge = Instance.DeadheadTemplates.Find(dht => dht.From == dt1.Trip.To && dht.To == chargeLoc);
                        DeadheadTemplate? dht_fromCharge = Instance.DeadheadTemplates.Find(dht => dht.From == chargeLoc && dht.To == dt2.Trip.From);

                        // No deadhead found / not feasible 
                        if (dht_toCharge == null || dht_fromCharge == null) continue;
                        int idleTime = dt2.Trip.StartTime - dt1.Trip.EndTime - dht_toCharge.Duration - dht_fromCharge.Duration;
                        if (idleTime < 0) continue;

                        // Deadhead feasible; see how much charge can be gained.
                        double SoCAtCharge = dt1.StartingSoC - SoCusedInDT1 - (dht_toCharge.Distance * vh.DriveUsage);
                        ChargingCurve cc = chargeLoc.ChargingCurves[vh.Id];
                        ChargeResult cr = cc.MaxChargeGained(SoCAtCharge, idleTime);

                        // Now, check if the charge is actually compatible; In order to do so, we need to check if we can use some partial charge such that our target SOC is reached.
                        double SoCDiff = dt2.StartingSoC - (SoCAtCharge - (dht_fromCharge.Duration * vh.DriveUsage));
                        double alpha = SoCDiff / cr.SoCGained;
                        if (alpha >= 0 && alpha <= 0)
                        {
                            // A feasible charge schedule was found; we now rerun the charge sequence in order to get costs
                            ChargeResult crUsed = cc.ChargeCosts(SoCAtCharge, SoCAtCharge + SoCDiff);

                            // Add deadhead that takes into account 2x driving costs and charging costs
                            // TODO: add time-dependent prices, staat nu gewoon vast

                            double drivingCosts = Config.KM_COST * (dht_toCharge.Distance + dht_fromCharge.Distance);
                            double chargingCost = crUsed.Cost;
                            feasibleDeadheads.Add((dt1, dt2, drivingCosts + chargingCost));
                            continue;
                        }
                    }
                }
            }

            // Create depot and its edges
            Location? depot = Instance.Locations.Find(loc => loc.IsDepot);
            if (depot == null) throw new InvalidDataException("At least one location should be flagged as a depot.");
            DiscreteDepot depotStart = new()
            {
                Id = discreteTrips.Sum(x => x.Count),
                Location = depot,
            };
            DiscreteDepot depotEnd = new()
            {
                Id = discreteTrips.Sum(x => x.Count) + 1,
                Location = depot,
            };

            // From depot to trip
            // TODO: depot to trip and vice versa can also do charging actions
            foreach (DiscreteTrip dt in discreteTrips.SelectMany(dts => dts.Select(x => x)))
            {
                // Check if there is a deadhead from the depot to the starting location of the trip
                DeadheadTemplate? dh = Instance.DeadheadTemplates.Find(dh => dh.From == depot && dh.To == dt.Trip.From);
                if (dh == null) throw new InvalidDataException("Can't go from the deadhead to a trip; This might be wrong");

                double SoCAtTrip = vh.StartCharge - (dh.Distance * vh.DriveUsage);
                double discretizedSoC = floorToDiscreteValue(SoCAtTrip);
                if (discretizedSoC == dt.StartingSoC)
                {
                    double drivingCosts = dh.Distance * Config.KM_COST;
                    double pullOutCosts = Config.PULLOUT_COST;
                    feasibleDeadheads.Add((depotStart, dt, drivingCosts + pullOutCosts));
                }
            }

            // From trip to depot
            foreach (DiscreteTrip dt in discreteTrips.SelectMany(dts => dts.Select(x => x)))
            {
                // Check if there is a deadhead from the depot to the starting location of the trip
                DeadheadTemplate? dh = Instance.DeadheadTemplates.Find(dh => dh.From == dt.Trip.To && dh.To == depot);
                if (dh == null) throw new InvalidDataException("Can't go from the deadhead to a trip; This might be wrong");

                double SoCAtDepot = dt.StartingSoC - ((dt.Trip.Distance + dh.Distance) * vh.DriveUsage);
                if (SoCAtDepot >= 0)
                {
                    double drivingCosts = dh.Distance * Config.KM_COST;
                    feasibleDeadheads.Add((dt, depotEnd, drivingCosts));
                }
            }


            // Using this, we can now create the model

            // Minimize the costs of all driven deadheads
            List<GRBVar> deadheadVars = new(feasibleDeadheads.Count);
            GRBLinExpr obj = new();
            for (int i = 0; i < feasibleDeadheads.Count; i++)
            {
                (var from, var to, var costs) = feasibleDeadheads[i];
                GRBVar v = model.AddVar(0, 1, 1, GRB.INTEGER, $"x_{from.Id},{to.Id}");
                deadheadVars.Add(v);
                obj.AddTerm(costs, v);
            }
            model.SetObjective(obj);

            // For each trip node, ensure that flow constraints are met.
            Dictionary<string, GRBLinExpr> tripFlowConstraints = new();
            Dictionary<string, GRBLinExpr> tripCoveredContraints = new();
            for (int i = 0; i < feasibleDeadheads.Count; i++)
            {
                (var from, var to, _) = feasibleDeadheads[i];
                GRBVar dhVar = deadheadVars[i];
                if (from is DiscreteTrip dtFrom)
                {
                    string key = dtFrom.Trip.Id + dtFrom.StartingSoC;
                    GRBLinExpr fromFlowExpr = tripFlowConstraints.ContainsKey(key)
                        ? tripFlowConstraints[key]
                        : new();
                    fromFlowExpr.AddTerm(-1, dhVar);
                }
                if (to is DiscreteTrip dtTo)
                {
                    string key = dtTo.Trip.Id + dtTo.StartingSoC;
                    GRBLinExpr fromFlowExpr = tripFlowConstraints.ContainsKey(key)
                        ? tripFlowConstraints[key]
                        : new();
                    fromFlowExpr.AddTerm(1, dhVar);

                    // Only use trip id as key here, as we dont care which soc a trip is covered at
                    GRBLinExpr fromCoverExpr = tripFlowConstraints.ContainsKey(dtTo.Trip.Id)
                        ? tripFlowConstraints[dtTo.Trip.Id]
                        : new();
                }
            }
            // Finalize by setting expressions equal to 0
            foreach ((string name, GRBLinExpr expr) in tripFlowConstraints)
            {
                model.AddConstr(expr, GRB.EQUAL, 0, "flow-" + name);
            }
            foreach ((string name, GRBLinExpr expr) in tripCoveredContraints)
            {
                model.AddConstr(expr, GRB.GREATER_EQUAL, 1, "cover-" + name);
            }

            // Lastly, ensure that depot trips are constrained. 
            // Max outgoing = n, ingoing = outgoing
            GRBLinExpr depotBalanced = new();
            GRBLinExpr depotOutgoingConstrained = new();
            for (int i = 0; i < feasibleDeadheads.Count; i++)
            {
                (var from, var to, _) = feasibleDeadheads[i];
                GRBVar dhVar = deadheadVars[i];
                if (from is DiscreteDepot dpFrom)
                {
                    depotBalanced.AddTerm(1, dhVar);
                    depotOutgoingConstrained.AddTerm(1, dhVar);
                }
                if (to is DiscreteDepot dpTo)
                {
                    depotBalanced.AddTerm(-1, dhVar);
                }
            }
            model.AddConstr(depotBalanced, GRB.EQUAL, 0, "depot_balanced");
            model.AddConstr(depotBalanced, GRB.LESS_EQUAL, Config.MAX_VEHICLES, "depot_balanced");

            // Solve
            model.Optimize();

            // Use model results.
            // TODO.
        }
    }
}

using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using E_VCSP.Solver;
using Microsoft.Msagl.Drawing;
using Color = Microsoft.Msagl.Drawing.Color;

namespace E_VCSP.Formatting
{
    public static class SolutionGraph
    {
        public static Graph GenerateVehicleTaskGraph(List<VehicleTask> tasks)
        {
            Graph graph = new();

            List<(int startTime, int endTime, List<Node?> nodes)> taskNodes = [];

            for (int i = 0; i < tasks.Count; i++)
            {
                var task = tasks[i];
                List<Node?> pathNodes = [];
                int startTime = task.Elements[0].StartTime, endTime = task.Elements[^1].EndTime;
                void add(Node? node)
                {
                    if (node != null)
                    {
                        graph.AddNode(node);
                        pathNodes.Add(node);
                    }
                }

                foreach (var element in task.Elements)
                {
                    string SoCAtStart = element.StartSoCInTask != null ? ((int)element.StartSoCInTask).ToString() : "";
                    string SoCAtEnd = element.EndSoCInTask != null ? ((int)element.EndSoCInTask).ToString() : "";

                    if (element is VEDepot) continue;
                    if (element is VETrip dvet)
                    {
                        Trip trip = dvet.Trip;
                        var node = Formatting.GraphElement.ScheduleNode(
                            element.StartTime,
                            element.EndTime,
                            $"{trip.From}@{SoCAtStart}% -> {trip.To}@{SoCAtEnd}% ({trip.Route})",
                            Color.LightBlue);
                        add(node);
                    }
                    if (element is VEIdle idle)
                    {
                        var node = Formatting.GraphElement.ScheduleNode(
                            element.StartTime,
                            element.EndTime,
                            $"idle {idle.Location.Id}@{SoCAtStart}% -> {SoCAtEnd}%",
                            Color.White);
                        add(node);
                    }
                    if (element is VEDeadhead dved)
                    {
                        Deadhead ddh = dved.Deadhead;

                        if (dved.SelectedAction == -1)
                        {
                            var node = Formatting.GraphElement.ScheduleNode(
                                element.StartTime,
                                element.EndTime,
                                $"{ddh.DeadheadTemplate.From}@{SoCAtStart}% -> {ddh.DeadheadTemplate.To}@{SoCAtEnd}%",
                                Color.LightGreen);
                            add(node);
                        }
                        else
                        {
                            ChargingAction ca = ddh.ChargingActions[dved.SelectedAction];
                            int currTime = element.StartTime;
                            double currSoC = element.StartSoCInTask ?? -10000000;
                            var toCharger = Formatting.GraphElement.ScheduleNode(
                                currTime,
                                currTime + ca.DrivingTimeTo,
                                $"{ddh.DeadheadTemplate.From}@{(int)currSoC} -> {ca.ChargeLocation}@{(int)(currSoC - ca.ChargeUsedTo)}",
                                Color.GreenYellow);
                            currTime += ca.DrivingTimeTo;
                            currSoC -= ca.ChargeUsedTo;
                            var charge = Formatting.GraphElement.ScheduleNode(
                                currTime,
                                currTime + dved.ChargeTime,
                                $"{ca.ChargeLocation} {(int)currSoC}% -> {(int)(currSoC + dved.SoCGained)}%",
                                Color.Yellow);
                            currTime += dved.ChargeTime;
                            currSoC += dved.SoCGained;
                            var fromCharger = Formatting.GraphElement.ScheduleNode(
                                currTime,
                                currTime + ca.DrivingTimeFrom,
                                $"{ca.ChargeLocation}@{(int)currSoC} -> {ddh.DeadheadTemplate.To}@{(int)(currSoC - ca.ChargeUsedFrom)}",
                                Color.GreenYellow);
                            add(toCharger);
                            add(charge);
                            add(fromCharger);
                        }
                    }
                }
                taskNodes.Add((startTime, endTime, pathNodes));
            }

            // Add padding to the start / end of each vehicle task in order align tasks
            int minTime = taskNodes.Min(x => x.startTime);
            int maxTime = taskNodes.Max(x => x.endTime);
            for (int i = 0; i < taskNodes.Count; i++)
            {
                (int s, int e, var ns) = taskNodes[i];
                var align = Formatting.GraphElement.ScheduleNode(
                    minTime - 1000,
                    minTime,
                    $"VT {tasks[i].Index}\nCosts: {tasks[i].Cost}",
                    Color.White);
                graph.AddNode(align);
                ns.Insert(0, align);

                if (minTime < s)
                {
                    var node = Formatting.GraphElement.ScheduleNode(minTime, s, "padding1" + i, Color.White);
                    graph.AddNode(node);
                    ns.Insert(1, node);
                }
                if (maxTime > e)
                {
                    var node = Formatting.GraphElement.ScheduleNode(e, maxTime, "padding2" + i, Color.White);
                    graph.AddNode(node);
                    ns.Add(node);
                }

                var align2 = Formatting.GraphElement.ScheduleNode(maxTime, maxTime + 300, "align2" + i, Color.White);
                graph.AddNode(align2);
                ns.Add(align2);
            }

            graph.LayoutAlgorithmSettings.NodeSeparation = 0;
            var lc = graph.LayerConstraints;
            lc.RemoveAllConstraints();

            // Force tasks to be on a single row
            foreach (var p in taskNodes) lc.AddSameLayerNeighbors(p.nodes);

            // This library isn't really built for alining nodes in a single layer;
            // force by centering nodes at beginning and end of each task
            List<Node?> leftAlign = [.. taskNodes.Select(x => x.nodes[0])],
                       rightAlign = [.. taskNodes.Select(x => x.nodes[^1])];
            for (int i = 0; i < leftAlign.Count - 1; i++) lc.AddUpDownVerticalConstraint(leftAlign[i], leftAlign[i + 1]);
            for (int i = 0; i < rightAlign.Count - 1; i++) lc.AddUpDownVerticalConstraint(rightAlign[i], rightAlign[i + 1]);
            return graph;
        }

        public static Graph GenerateBlockGraph(List<List<Block>> tasks)
        {
            Graph graph = new();

            List<(int startTime, int endTime, List<Node?> nodes)> taskNodes = [];

            for (int i = 0; i < tasks.Count; i++)
            {
                var task = tasks[i];
                List<Node?> pathNodes = [];
                int startTime = task[0].StartTime, endTime = task[^1].EndTime;
                void add(Node? node)
                {
                    if (node != null)
                    {
                        graph.AddNode(node);
                        pathNodes.Add(node);
                    }
                }

                for (int j = 0; j < task.Count; j++)
                {
                    Block block = task[j];
                    var node = Formatting.GraphElement.ScheduleNode(
                        block.StartTime,
                        block.EndTime,
                        $"{block.From} -> {block.To}",
                        Color.LightBlue
                    );
                    add(node);

                    if (j < task.Count - 1 && block.EndTime < task[j + 1].StartTime)
                    {
                        // Idle
                        var idleNode = Formatting.GraphElement.ScheduleNode(
                            block.EndTime,
                            task[j + 1].StartTime,
                            $"IDLE",
                            Color.White
                        );
                        add(idleNode);
                    }
                }
                taskNodes.Add((startTime, endTime, pathNodes));
            }

            // Add padding to the start / end of each vehicle task in order align tasks
            int minTime = taskNodes.Min(x => x.startTime);
            int maxTime = taskNodes.Max(x => x.endTime);
            for (int i = 0; i < taskNodes.Count; i++)
            {
                (int s, int e, var ns) = taskNodes[i];
                var align = Formatting.GraphElement.ScheduleNode(
                    minTime - 1000,
                    minTime,
                    $"Task {i}",
                    Color.White);
                graph.AddNode(align);
                ns.Insert(0, align);

                if (minTime < s)
                {
                    var node = Formatting.GraphElement.ScheduleNode(minTime, s, "padding1" + i, Color.White);
                    graph.AddNode(node);
                    ns.Insert(1, node);
                }
                if (maxTime > e)
                {
                    var node = Formatting.GraphElement.ScheduleNode(e, maxTime, "padding2" + i, Color.White);
                    graph.AddNode(node);
                    ns.Add(node);
                }

                var align2 = Formatting.GraphElement.ScheduleNode(maxTime, maxTime + 300, "align2" + i, Color.White);
                graph.AddNode(align2);
                ns.Add(align2);
            }

            graph.LayoutAlgorithmSettings.NodeSeparation = 0;
            var lc = graph.LayerConstraints;
            lc.RemoveAllConstraints();

            // Force tasks to be on a single row
            foreach (var p in taskNodes) lc.AddSameLayerNeighbors(p.nodes);

            // This library isn't really built for alining nodes in a single layer;
            // force by centering nodes at beginning and end of each task
            List<Node?> leftAlign = [.. taskNodes.Select(x => x.nodes[0])],
                       rightAlign = [.. taskNodes.Select(x => x.nodes[^1])];
            for (int i = 0; i < leftAlign.Count - 1; i++) lc.AddUpDownVerticalConstraint(leftAlign[i], leftAlign[i + 1]);
            for (int i = 0; i < rightAlign.Count - 1; i++) lc.AddUpDownVerticalConstraint(rightAlign[i], rightAlign[i + 1]);
            return graph;
        }
    }
}

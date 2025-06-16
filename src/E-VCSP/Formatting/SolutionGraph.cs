using E_VCSP.Objects;
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
                    string SoCAtStart = ((int)element.StartSoCInTask).ToString();
                    string SoCAtEnd = ((int)element.EndSoCInTask).ToString();

                    Color color = Color.Red;
                    string text = "Forgot a case";

                    if (element is VEIdle vei)
                    {
                        text = $"idle {vei.StartLocation}@{SoCAtStart}% -> {SoCAtEnd}%";
                        color = Color.White;
                    }
                    else if (element is VETrip vet)
                    {
                        color = Color.LightBlue;
                        text = $"{vet.Trip.From}@{SoCAtStart}% -> {vet.Trip.To}@{SoCAtEnd}% ({vet.Trip.Route})";
                    }
                    else if (element is VEDeadhead ved)
                    {
                        text = $"{ved.DeadheadTemplate.From}@{SoCAtStart}% -> {ved.DeadheadTemplate.To}@{SoCAtEnd}%";
                        color = Color.LightGreen;
                    }
                    else if (element is VECharge vec)
                    {
                        text = $"{vec.StartLocation} {SoCAtStart}% -> {SoCAtEnd}%";
                        color = Color.Yellow;
                    }
                    Node? node = GraphElement.ScheduleNode(
                        element.StartTime,
                        element.EndTime,
                        text,
                        color
                    );
                    add(node);
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

        public static Graph GenerateBlockGraph(List<List<Block>> blockRows)
        {
            Graph graph = new();

            List<(int startTime, int endTime, List<Node?> nodes)> taskNodes = [];

            for (int i = 0; i < blockRows.Count; i++)
            {
                var task = blockRows[i];
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
                        $"{block.StartLocation} -> {block.EndLocation}",
                        block.EndTime - block.StartTime > Config.MAX_DRIVE_TIME ? Color.Red : Color.LightBlue
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

        public static Graph GenerateCrewDutyGraph(List<CrewDuty> duties)
        {
            Graph graph = new();

            List<(int startTime, int endTime, List<Node?> nodes)> taskNodes = [];

            for (int i = 0; i < duties.Count; i++)
            {
                var duty = duties[i];
                List<Node?> rowNodes = [];
                int startTime = duty.Elements[0].StartTime, endTime = duty.Elements[^1].EndTime;

                void add(Node? node)
                {
                    if (node != null)
                    {
                        graph.AddNode(node);
                        rowNodes.Add(node);
                    }
                }

                foreach (var element in duty.Elements)
                {
                    Color color = Color.Red;
                    string text = "Forgot a case!";

                    if (element is CDEIdle cdei)
                    {
                        text = $"idle {cdei.StartLocation}";
                        color = Color.White;
                    }
                    else if (element is CDEBlock cdebl)
                    {
                        color = Color.LightBlue;
                        text = $"{cdebl.StartLocation} -> {cdebl.EndLocation}";
                    }
                    else if (element is CDETravel cdet)
                    {
                        text = $"{cdet.StartLocation} -> {cdet.EndLocation}";
                        color = Color.LightGreen;
                    }
                    else if (element is CDEBreak cdebr)
                    {
                        text = $"{cdebr.StartLocation}";
                        color = Color.DarkSalmon;
                    }
                    Node? node = GraphElement.ScheduleNode(
                        element.StartTime,
                        element.EndTime,
                        text,
                        color
                    );
                    add(node);
                }
                taskNodes.Add((startTime, endTime, rowNodes));
            }

            int minTime = taskNodes.Min(x => x.startTime);
            int maxTime = taskNodes.Max(x => x.endTime);
            for (int i = 0; i < taskNodes.Count; i++)
            {
                (int s, int e, var ns) = taskNodes[i];
                var align = Formatting.GraphElement.ScheduleNode(
                    minTime - 1000,
                    minTime,
                    $"Duty {duties[i].Index}\nCosts: {duties[i].Cost}",
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

                var align2 = Formatting.GraphElement.ScheduleNode(maxTime, maxTime + Config.MIN_NODE_TIME, "align2" + i, Color.White);
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

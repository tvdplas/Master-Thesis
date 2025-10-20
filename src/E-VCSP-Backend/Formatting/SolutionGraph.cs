using E_VCSP.Objects;
using E_VCSP_Backend.Formatting;

namespace E_VCSP.Formatting {
    public static class SolutionGraph {
        public static List<List<RosterNode>> GenerateVehicleTaskGraph(List<VehicleTask> tasks) {
            List<List<RosterNode>> rosterNodes = [];
            tasks = tasks.OrderBy(x => x.Elements[0].StartTime).ToList();
            Dictionary<int, int> tripCount = [];
            for (int i = 0; i < tasks.Count; i++) {
                VehicleTask vt = tasks[i];
                for (int j = 0; j < vt.Elements.Count; j++) {
                    if (vt.Elements[j].Type == VEType.Trip) {
                        int ti = ((VETrip)vt.Elements[j]).Trip.Index;
                        tripCount.TryAdd(ti, 0);
                        tripCount[ti]++;
                    }
                }
            }

            for (int i = 0; i < tasks.Count; i++) {
                var task = tasks[i];
                List<RosterNode> taskNodes = [];

                foreach (var element in task.Elements) {
                    string SoCAtStart = ((int)element.StartSoCInTask).ToString();
                    string SoCAtEnd = ((int)element.EndSoCInTask).ToString();

                    string color = "#FF0000";
                    string content = "Forgot a case";

                    if (element is VEIdle vei) {
                        if (element.Postprocessed) {
                            content = $"POST idle {vei.StartLocation}@{SoCAtStart}% -> {SoCAtEnd}%";
                            color = "#D3D3D3";
                        }
                        else {
                            content = $"idle {vei.StartLocation}@{SoCAtStart}% -> {SoCAtEnd}%";
                            color = "#FFFFFF";
                        }
                    }
                    else if (element is VETrip vet) {
                        color = tripCount[vet.Trip.Index] == 1 ? "#ADD8E6" : "#77A6B5";
                        content = $"{vet.Trip.StartLocation}@{SoCAtStart}% -> {vet.Trip.EndLocation}@{SoCAtEnd}% ({vet.Trip.Route} / {vet.Trip.Id})";
                    }
                    else if (element is VEDeadhead ved) {
                        if (element.Postprocessed) {
                            content = $"POST {ved.DeadheadTemplate.StartLocation}@{SoCAtStart}% -> {ved.DeadheadTemplate.EndLocation}@{SoCAtEnd}%";
                            color = "#808000";
                        }
                        else {
                            content = $"{ved.DeadheadTemplate.StartLocation}@{SoCAtStart}% -> {ved.DeadheadTemplate.EndLocation}@{SoCAtEnd}%";
                            color = "#90EE90";
                        }
                    }
                    else if (element is VECharge vec) {
                        content = $"{vec.StartLocation} {SoCAtStart}% -> {SoCAtEnd}%";
                        color = "#FFFF00";
                    }
                    RosterNode rosterNode = new() {
                        StartTime = element.StartTime,
                        EndTime = element.EndTime,
                        Content = content,
                        ColorHex = color
                    };
                    taskNodes.Add(rosterNode);
                }

                taskNodes.Add(new RosterNode() {
                    StartTime = tasks.Min(x => x.Elements[0].StartTime) - 1000,
                    EndTime = tasks.Min(x => x.Elements[0].StartTime),
                    Content = $"VT {task.Index}\n Source: {task.Source}",
                    ColorHex = "#808080"
                });

                rosterNodes.Add(taskNodes);
            }

            return rosterNodes;
        }

        public static List<List<RosterNode>> GenerateBlockGraph(List<List<Block>> blockRows) {
            blockRows = blockRows.OrderBy(x => x[0].StartTime).ToList();

            Dictionary<int, int> blockCount = [];
            for (int i = 0; i < blockRows.Count; i++) {
                List<Block> blocks = blockRows[i];
                for (int j = 0; j < blocks.Count; j++) {
                    int bi = blocks[j].Index;
                    blockCount.TryAdd(bi, 0);
                    blockCount[bi]++;
                }
            }

            List<List<RosterNode>> taskNodes = [];

            for (int i = 0; i < blockRows.Count; i++) {
                var task = blockRows[i];
                List<RosterNode> pathNodes = [];
                for (int j = 0; j < task.Count; j++) {
                    Block block = task[j];
                    string blockColor = "#ADD8E6";
                    if (block.Index != -1 && blockCount[block.Index] != 1) blockColor = "#77A6B5";
                    if (block.EndTime - block.StartTime > Constants.MAX_STEERING_TIME) blockColor = "#FF0000";

                    pathNodes.Add(new RosterNode() {
                        StartTime = block.StartTime,
                        EndTime = block.EndTime,
                        Content = $"{block.StartLocation} -> {block.EndLocation} ({block.Index})",
                        ColorHex = blockColor
                    });

                    if (j + 1 < task.Count && block.EndTime < task[j + 1].StartTime) {
                        pathNodes.Add(new RosterNode() {
                            StartTime = block.EndTime,
                            EndTime = task[j + 1].StartTime,
                            Content = "IDLE",
                            ColorHex = "#FFFFFF"
                        });
                    }
                }

                pathNodes.Add(new RosterNode() {
                    StartTime = blockRows[0][0].StartTime - 1000,
                    EndTime = blockRows[0][0].StartTime,
                    Content = $"Task {i}",
                    ColorHex = "#FFFFFF"
                });

                taskNodes.Add(pathNodes);
            }

            return taskNodes;
        }

        public static List<List<RosterNode>> GenerateCrewDutyGraph(List<(CrewDuty duty, int count)> dutiesWithCount) {
            List<CrewDuty> duties = dutiesWithCount
                .SelectMany(x => Enumerable.Repeat(x.duty, x.count))
                .OrderBy(x => x.Elements[0].StartTime).ToList();

            Dictionary<int, int> blockCount = [];
            for (int i = 0; i < duties.Count; i++) {
                var elems = duties[i].Elements;
                for (int j = 0; j < elems.Count; j++) {
                    if (elems[j] is CDEBlock cdeb) {
                        int bi = cdeb.Block.Index;
                        blockCount.TryAdd(bi, 0);
                        blockCount[bi]++;
                    }
                }
            }

            List<List<RosterNode>> taskNodes = [];

            for (int i = 0; i < duties.Count; i++) {
                var duty = duties[i];
                List<RosterNode> rowNodes = [];

                foreach (var element in duty.Elements) {
                    if (element is CDEBreak cdebr) {
                        int walkTime = element.StartLocation.BrutoNetto / 2;

                        rowNodes.Add(new RosterNode() {
                            StartTime = element.StartTime,
                            EndTime = element.StartTime + walkTime,
                            Content = "walk",
                            ColorHex = "#FFA07A"
                        });
                        rowNodes.Add(new RosterNode() {
                            StartTime = element.StartTime + walkTime,
                            EndTime = element.EndTime - walkTime,
                            Content = $"{cdebr.StartLocation}",
                            ColorHex = "#E9967A"
                        });
                        rowNodes.Add(new RosterNode() {
                            StartTime = element.EndTime - walkTime,
                            EndTime = element.EndTime,
                            Content = "walk",
                            ColorHex = "#FFA07A"
                        });
                    }
                    else {
                        string color = "#FF0000";
                        string content = "Forgot a case!";
                        if (element is CDEIdle cdei) {
                            int totalTime = cdei.EndTime - cdei.StartTime;
                            content = $"idle {cdei.StartLocation}";
                            color = totalTime > Config.CR_MIN_LONG_IDLE_TIME ? "#F5FFFA" : "#FFFFFF";
                        }
                        else if (element is CDEBlock cdebl) {
                            color = "#ADD8E6";
                            color = blockCount[cdebl.Block.Index] == 1 ? "#ADD8E6" : "#77A6B5";
                            content = $"{cdebl.StartLocation} -> {cdebl.EndLocation} ({cdebl.Block.Index})";
                        }
                        else if (element is CDETravel cdet) {
                            content = $"{cdet.StartLocation} -> {cdet.EndLocation}";
                            color = "#90EE90";
                        }
                        else if (element is CDESignOnOff cdes) {
                            content = $"{cdes.StartLocation}";
                            color = "#E6E6FA";
                        }

                        rowNodes.Add(new RosterNode() {
                            StartTime = element.StartTime,
                            EndTime = element.EndTime,
                            Content = content,
                            ColorHex = color
                        });
                    }

                }

                string shiftTypeColor = "#FFC0CB";
                if (duties[i].Type == DutyType.Early) shiftTypeColor = "#FFFF00";
                else if (duties[i].Type == DutyType.Day) shiftTypeColor = "#ADD8E6";
                else if (duties[i].Type == DutyType.Broken) shiftTypeColor = "#ADFF2F";
                else if (duties[i].Type == DutyType.Between) shiftTypeColor = "#00FF00";
                else if (duties[i].Type == DutyType.Late) shiftTypeColor = "#0000FF";
                else if (duties[i].Type == DutyType.Night) shiftTypeColor = "#8A2BE2";
                else if (duties[i].Type == DutyType.Single) shiftTypeColor = "#FF0000";

                rowNodes.Add(new RosterNode() {
                    StartTime = duties[0].Elements[0].StartTime - 1000,
                    EndTime = duties[0].Elements[0].StartTime,
                    Content = $"{duties[i]} {Formatting.Time.HHMMSS(duties[i].Elements[0].StartTime)}-{Formatting.Time.HHMMSS(duties[i].Elements[^1].EndTime)}\nCosts: {duties[i].Cost}",
                    ColorHex = shiftTypeColor
                });

                taskNodes.Add(rowNodes);
            }

            return taskNodes;
        }
    }
}

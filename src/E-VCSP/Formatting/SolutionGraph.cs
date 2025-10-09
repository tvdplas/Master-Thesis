using E_VCSP.Objects;

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

                    System.Drawing.Color color = System.Drawing.Color.Red;
                    string content = "Forgot a case";

                    if (element is VEIdle vei) {
                        if (element.Postprocessed) {
                            content = $"POST idle {vei.StartLocation}@{SoCAtStart}% -> {SoCAtEnd}%";
                            color = System.Drawing.Color.LightGray;
                        }
                        else {
                            content = $"idle {vei.StartLocation}@{SoCAtStart}% -> {SoCAtEnd}%";
                            color = System.Drawing.Color.White;
                        }
                    }
                    else if (element is VETrip vet) {
                        color = tripCount[vet.Trip.Index] == 1 ? System.Drawing.Color.LightBlue : Color.FromArgb(119, 166, 181);
                        content = $"{vet.Trip.StartLocation}@{SoCAtStart}% -> {vet.Trip.EndLocation}@{SoCAtEnd}% ({vet.Trip.Route} / {vet.Trip.Id})";
                    }
                    else if (element is VEDeadhead ved) {
                        if (element.Postprocessed) {
                            content = $"POST {ved.DeadheadTemplate.StartLocation}@{SoCAtStart}% -> {ved.DeadheadTemplate.EndLocation}@{SoCAtEnd}%";
                            color = System.Drawing.Color.Olive;
                        }
                        else {
                            content = $"{ved.DeadheadTemplate.StartLocation}@{SoCAtStart}% -> {ved.DeadheadTemplate.EndLocation}@{SoCAtEnd}%";
                            color = System.Drawing.Color.LightGreen;
                        }
                    }
                    else if (element is VECharge vec) {
                        content = $"{vec.StartLocation} {SoCAtStart}% -> {SoCAtEnd}%";
                        color = System.Drawing.Color.Yellow;
                    }
                    RosterNode rosterNode = new() {
                        StartTime = element.StartTime,
                        EndTime = element.EndTime,
                        Content = content,
                        Color = color
                    };
                    taskNodes.Add(rosterNode);
                }

                taskNodes.Add(new RosterNode() {
                    StartTime = tasks.Min(x => x.Elements[0].StartTime) - 1000,
                    EndTime = tasks.Min(x => x.Elements[0].StartTime),
                    Content = $"VT {task.Index}\n Source: {task.Source}",
                    Color = System.Drawing.Color.Gray
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
                    Color blockColor = Color.LightBlue;
                    if (block.Index != -1 && blockCount[block.Index] != 1) blockColor = Color.FromArgb(119, 166, 181);
                    if (block.EndTime - block.StartTime > Constants.MAX_STEERING_TIME) blockColor = Color.Red;

                    pathNodes.Add(new RosterNode() {
                        StartTime = block.StartTime,
                        EndTime = block.EndTime,
                        Content = $"{block.StartLocation} -> {block.EndLocation} ({block.Index})",
                        Color = blockColor
                    });

                    if (j + 1 < task.Count && block.EndTime < task[j + 1].StartTime) {
                        pathNodes.Add(new RosterNode() {
                            StartTime = block.EndTime,
                            EndTime = task[j + 1].StartTime,
                            Content = "IDLE",
                            Color = Color.White
                        });
                    }
                }

                pathNodes.Add(new RosterNode() {
                    StartTime = blockRows[0][0].StartTime - 1000,
                    EndTime = blockRows[0][0].StartTime,
                    Content = $"Task {i}",
                    Color = Color.White
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
                            Color = Color.LightSalmon
                        });
                        rowNodes.Add(new RosterNode() {
                            StartTime = element.StartTime + walkTime,
                            EndTime = element.EndTime - walkTime,
                            Content = $"{cdebr.StartLocation}",
                            Color = Color.DarkSalmon
                        });
                        rowNodes.Add(new RosterNode() {
                            StartTime = element.EndTime - walkTime,
                            EndTime = element.EndTime,
                            Content = "walk",
                            Color = Color.LightSalmon
                        });
                    }
                    else {
                        Color color = Color.Red;
                        string content = "Forgot a case!";
                        if (element is CDEIdle cdei) {
                            int totalTime = cdei.EndTime - cdei.StartTime;
                            content = $"idle {cdei.StartLocation}";
                            color = totalTime > Constants.CR_MIN_LONG_IDLE_TIME ? Color.MintCream : Color.White;
                        }
                        else if (element is CDEBlock cdebl) {
                            color = Color.LightBlue;
                            color = blockCount[cdebl.Block.Index] == 1 ? Color.LightBlue : Color.FromArgb(119, 166, 181);
                            content = $"{cdebl.StartLocation} -> {cdebl.EndLocation} ({cdebl.Block.Index})";
                        }
                        else if (element is CDETravel cdet) {
                            content = $"{cdet.StartLocation} -> {cdet.EndLocation}";
                            color = Color.LightGreen;
                        }
                        else if (element is CDESignOnOff cdes) {
                            content = $"{cdes.StartLocation}";
                            color = Color.Lavender;
                        }

                        rowNodes.Add(new RosterNode() {
                            StartTime = element.StartTime,
                            EndTime = element.EndTime,
                            Content = content,
                            Color = color
                        });
                    }

                }

                Color shiftTypeColor = Color.Pink;
                if (duties[i].Type == DutyType.Early) shiftTypeColor = Color.Yellow;
                else if (duties[i].Type == DutyType.Day) shiftTypeColor = Color.LightBlue;
                else if (duties[i].Type == DutyType.Broken) shiftTypeColor = Color.GreenYellow;
                else if (duties[i].Type == DutyType.Between) shiftTypeColor = Color.Green;
                else if (duties[i].Type == DutyType.Late) shiftTypeColor = Color.Blue;
                else if (duties[i].Type == DutyType.Night) shiftTypeColor = Color.BlueViolet;
                else if (duties[i].Type == DutyType.Single) shiftTypeColor = Color.Red;

                rowNodes.Add(new RosterNode() {
                    StartTime = duties[0].Elements[0].StartTime - 1000,
                    EndTime = duties[0].Elements[0].StartTime,
                    Content = $"{duties[i]} {Formatting.Time.HHMMSS(duties[i].Elements[0].StartTime)}-{Formatting.Time.HHMMSS(duties[i].Elements[^1].EndTime)}\nCosts: {duties[i].Cost}",
                    Color = shiftTypeColor
                });

                taskNodes.Add(rowNodes);
            }

            return taskNodes;
        }
    }
}

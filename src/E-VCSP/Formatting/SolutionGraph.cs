using E_VCSP.Objects;

namespace E_VCSP.Formatting {
    public static class SolutionGraph {
        public static List<List<RosterNode>> GenerateVehicleTaskGraph(List<VehicleTask> tasks) {

            List<List<RosterNode>> rosterNodes = [];
            tasks = tasks.OrderBy(x => x.Elements[0].StartTime).ToList();

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
                        color = System.Drawing.Color.LightBlue;
                        content = $"{vet.Trip.StartLocation}@{SoCAtStart}% -> {vet.Trip.EndLocation}@{SoCAtEnd}% ({vet.Trip.Route} / {vet.Trip.Id})";
                    }
                    else if (element is VEDeadhead ved) {
                        if (element.Postprocessed) {
                            content = $"POST {ved.DeadheadTemplate.StartLocation}@{SoCAtStart}% -> {ved.DeadheadTemplate.To}@{SoCAtEnd}%";
                            color = System.Drawing.Color.Olive;
                        }
                        else {
                            content = $"{ved.DeadheadTemplate.StartLocation}@{SoCAtStart}% -> {ved.DeadheadTemplate.To}@{SoCAtEnd}%";
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

            List<List<RosterNode>> taskNodes = [];

            for (int i = 0; i < blockRows.Count; i++) {
                var task = blockRows[i];
                List<RosterNode> pathNodes = [];
                for (int j = 0; j < task.Count; j++) {
                    Block block = task[j];
                    pathNodes.Add(new RosterNode() {
                        StartTime = block.StartTime,
                        EndTime = block.EndTime,
                        Content = $"{block.StartLocation} -> {block.EndLocation} ({block.Index})",
                        Color = block.EndTime - block.StartTime > Constants.MAX_STEERING_TIME ? Color.Red : Color.LightBlue
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

        public static List<List<RosterNode>> GenerateCrewDutyGraph(List<CrewDuty> duties) {
            duties = duties.OrderBy(x => x.Elements[0].StartTime).ToList();

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

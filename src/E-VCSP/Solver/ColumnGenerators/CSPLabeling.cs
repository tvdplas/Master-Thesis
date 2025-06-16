using E_VCSP.Objects;

namespace E_VCSP.Solver.ColumnGenerators
{
    internal class BlockArc
    {
        internal Block? FromBlock;
        internal Block? ToBlock;
        internal int BreakTime;
        internal int BruttoNettoTime;
        internal int IdleTime;
        internal int TravelTime;

        internal int TotalTime => BreakTime + BruttoNettoTime + IdleTime + TravelTime;
    }

    internal class CSPLabel
    {
        private static int ID_COUNTER = 0;
        internal readonly int Id = -1;
        internal CSPLabel()
        {
            Id = ID_COUNTER++;
        }

        internal int PrevLabelId = -1;
        internal int PrevBlockId = -1;
        internal DutyType Type;
        internal double Cost = 0;
        internal int Duration = 0;
        internal List<int> Breaks = new();
        internal List<int> Idles = new();

        internal bool isFeasible(int currentEndTime)
        {
            if (Duration > 9 * 60 * 60) return false;
            if (Type == DutyType.Early && currentEndTime > 16.5 * 60 * 60) return false;
            if (Type == DutyType.Day && currentEndTime > 18.25 * 60 * 60) return false;
            if (Type == DutyType.Late && (currentEndTime - Duration < 13 * 60 * 60 || currentEndTime > 26.5 * 60 * 60)) return false;
            if (Type == DutyType.Night && (Duration > 7 * 60 * 60 || currentEndTime - Duration > 24 * 60 * 60)) return false;
            if (Type == DutyType.Between && currentEndTime - Duration > 13 * 60 * 60) return false;
            if (Type == DutyType.Broken && (currentEndTime - Duration < 5.5 * 60 * 60 || currentEndTime > 19.5 * 60 * 60)) return false;

            // TODO: breaks
            return true;
        }
    }

    internal class CSPLabeling
    {
        private List<Block> blocks;

        List<List<BlockArc?>> adjFull;
        List<List<BlockArc>> adj;

        internal CSPLabeling(List<Block> blocks)
        {
            this.blocks = blocks;

            adjFull = Enumerable.Range(0, blocks.Count + 2).Select(x => new List<BlockArc?>()).ToList();
            adj = Enumerable.Range(0, blocks.Count + 2).Select(x => new List<BlockArc>()).ToList();

            // TODO: add block arcs for walking
            for (int blockIndex1 = 0; blockIndex1 < blocks.Count; blockIndex1++)
            {
                Block block1 = blocks[blockIndex1];
                for (int blockIndex2 = 0; blockIndex2 < blocks.Count; blockIndex2++)
                {
                    Block block2 = blocks[blockIndex2];

                    // Determine whether or not it is feasible to string to arcs together
                    // If so: what actually happens during this time. 

                    // For now; only allow transfer if already at same location
                    // Based on times, determine whether its idle / break / whatever. 
                    BlockArc? arc = null;

                    if (block1.EndTime <= block2.StartTime && block1.EndLocation == block2.StartLocation)
                    {
                        // Arc will be formed; need to fill in details.
                        int idleTime = block2.StartTime - block1.EndTime;
                        int breakTime = 0;
                        int bruttoNettoTime = 0;
                        int travelTime = 0;

                        if (block1.EndLocation.BreakAllowed && idleTime > block1.EndLocation.BrutoNetto)
                        {
                            breakTime = idleTime - block1.EndLocation.BrutoNetto;
                            bruttoNettoTime = block1.EndLocation.BrutoNetto;
                            idleTime = 0;
                        }

                        arc = new()
                        {
                            FromBlock = block1,
                            ToBlock = block2,
                            IdleTime = idleTime,
                            BreakTime = breakTime,
                            BruttoNettoTime = bruttoNettoTime,
                            TravelTime = travelTime,
                        };
                    }

                    if (arc != null) adj[blockIndex1].Add(arc);
                    adjFull[blockIndex1].Add(arc);
                }
            }

            // TODO: better depot handling
            for (int blockIndex = 0; blockIndex < blocks.Count; blockIndex++)
            {
                Block block = blocks[blockIndex];
                BlockArc? start = block.StartLocation.IsDepot ? new BlockArc()
                {
                    ToBlock = block,
                    IdleTime = 0,
                    BreakTime = 0,
                    BruttoNettoTime = block.StartLocation.BrutoNetto,
                    TravelTime = 0,
                } : null;
                if (start != null) adj[^2].Add(start);
                adjFull[^2].Add(start);
                adjFull[blockIndex].Add(null); // cannot go back to depot

                BlockArc? end = block.EndLocation.IsDepot ? new BlockArc()
                {
                    ToBlock = block,
                    IdleTime = 0,
                    BreakTime = 0,
                    BruttoNettoTime = block.EndLocation.BrutoNetto,
                    TravelTime = 0,
                } : null;
                if (end != null) adj[block.Index].Add(end);
                adjFull[block.Index].Add(end);
            }
        }

        internal (double reducedCosts, CrewDuty crewDuty) generateShortestPath(List<double> reducedCosts)
        {
            List<List<CSPLabel>> allLabels = [.. Enumerable.Range(0, blocks.Count + 2).Select(x => new List<CSPLabel>())];
            List<List<CSPLabel>> activeLabels = [.. Enumerable.Range(0, blocks.Count + 2).Select(x => new List<CSPLabel>())];

            void addLabel(CSPLabel l, int index)
            {
                allLabels[index].Add(l);
                activeLabels[index].Add(l);
            }

            for (int i = 0; i < (int)DutyType.Count; i++)
            {
                DutyType dt = (DutyType)i;

                // Start by setting out all possible duty types from the depot start
                for (int targetBlockId = 0; targetBlockId < blocks.Count; targetBlockId++)
                {
                    BlockArc? arc = adjFull[^2][targetBlockId];
                    if (arc == null) continue;

                    addLabel(new()
                    {
                        PrevLabelId = -1,
                        PrevBlockId = -1,
                        Type = dt,
                        Cost = dt == DutyType.Broken ? Config.CREW_BROKEN_SHIFT_COST : 0,
                        Duration = 0,
                        Breaks = [],
                        Idles = [],
                    }, arc.ToBlock!.Index);
                }
            }

            while (activeLabels.Where((x, i) => i <= blocks.Count).Sum(x => x.Count) > 0)
            {
                // Find node with active label
                CSPLabel? currentLabel = null;
                int currentNodeIndex = -1;
                for (int i = 0; i < activeLabels.Count - 1; i++)
                {
                    if (activeLabels[i].Count > 0)
                    {
                        currentLabel = activeLabels[i][^1];
                        activeLabels.RemoveAt(activeLabels.Count - 1);
                        currentNodeIndex = i;
                        break;
                    }
                }
                if (currentLabel == null) throw new InvalidOperationException("heh");

                // Expand node if possible 
                for (int targetBlockIndex = 0; targetBlockIndex < blocks.Count + 2; targetBlockIndex++)
                {
                    if (targetBlockIndex == blocks.Count) continue;

                    BlockArc? arc = adjFull[currentNodeIndex][targetBlockIndex];
                    if (arc == null) continue;

                    CSPLabel newLabel = new()
                    {
                        Breaks = [.. currentLabel.Breaks, arc.BreakTime],
                        Idles = [.. currentLabel.Idles, arc.IdleTime],
                        Duration = currentLabel.Duration + arc.TotalTime,
                        PrevBlockId = currentNodeIndex,
                        Cost = currentLabel.Cost + (targetBlockIndex < blocks.Count ? reducedCosts[targetBlockIndex] : 0),
                        PrevLabelId = currentLabel.Id,
                        Type = currentLabel.Type,
                    };

                    if (!newLabel.isFeasible(blocks[targetBlockIndex >= blocks.Count ? currentNodeIndex : targetBlockIndex].EndTime)) continue;

                    // TODO: domination rules
                    else addLabel(newLabel, targetBlockIndex);
                }
            }

            Console.WriteLine("Done with labeling");

            throw new InvalidOperationException();
        }
    }
}
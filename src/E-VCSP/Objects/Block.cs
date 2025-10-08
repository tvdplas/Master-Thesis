using E_VCSP.Objects.ParsedData;
using E_VCSP.Utils;
using System.Text.Json.Serialization;

namespace E_VCSP.Objects {
    public enum BlockElementType {
        Idle,
        Trip,
        Deadhead,
        Charge,
    }
    public class BlockElement {
        [JsonInclude]
        public int StartTime;
        [JsonInclude]
        public int EndTime;
        [JsonInclude]
        public Location StartLocation;
        [JsonInclude]
        public Location EndLocation;
        [JsonInclude]
        public BlockElementType Type;

        public static List<BlockElement> FromVE(VehicleElement ve) {
            List<BlockElement> elements = new();

            if (ve is VEIdle vei) {
                elements.Add(new() {
                    Type = BlockElementType.Idle,
                    EndTime = ve.EndTime,
                    StartTime = ve.StartTime,
                    StartLocation = vei.StartLocation!,
                    EndLocation = vei.EndLocation!,
                });
            }
            else if (ve is VETrip vet) {
                elements.Add(new BETrip() {
                    Type = BlockElementType.Trip,
                    Trip = vet.Trip,
                    StartTime = vet.StartTime,
                    EndTime = vet.EndTime,
                    StartLocation = vet.Trip.StartLocation,
                    EndLocation = vet.Trip.EndLocation,
                });
            }
            else if (ve is VEDeadhead ved) {
                // Edge case for self arc "deadhead" 
                if (ved.DeadheadTemplate.Duration > 0) {
                    elements.Add(new BEDeadhead() {
                        DeadheadTemplate = ved.DeadheadTemplate,
                        EndTime = ved.EndTime,
                        StartTime = ved.StartTime,
                        StartLocation = ved.DeadheadTemplate.StartLocation,
                        EndLocation = ved.DeadheadTemplate.EndLocation,
                        Type = BlockElementType.Deadhead,
                    });
                }
            }
            else if (ve is VECharge vec) {
                elements.Add(new BlockElement() {
                    EndTime = vec.EndTime,
                    StartTime = vec.StartTime,
                    StartLocation = vec.StartLocation!,
                    EndLocation = vec.EndLocation!,
                    Type = BlockElementType.Charge,
                });
            }
            else {
                throw new InvalidDataException("Forgot to add VE case in conversion to BE");
            }

            return elements;
        }
    }

    public class BETrip : BlockElement {
        public required Trip Trip;
    }

    public class BEDeadhead : BlockElement {
        public required DeadheadTemplate DeadheadTemplate;
    }

    /// <summary>
    /// JSON dump form of a block
    /// </summary>
    public class BlockDump {
        [JsonInclude]
        public required Location StartLocation;
        [JsonInclude]
        public required Location EndLocation;
        [JsonInclude]
        public int StartTime;
        [JsonInclude]
        public int EndTime;

        public static BlockDump FromBlock(Block b) {
            return new BlockDump() {
                StartLocation = b.StartLocation,
                EndLocation = b.EndLocation,
                StartTime = b.StartTime,
                EndTime = b.EndTime,
            };
        }
    }

    /// <summary>
    /// Part of a vehicle task / crew schedule
    /// </summary>
    public class Block {
        static List<BlockElementType> IDLE_TYPES = [BlockElementType.Idle, BlockElementType.Charge];

        [JsonInclude]
        public List<BlockElement> Elements = new();
        public int Index = -1;

        public Location StartLocation {
            get => Elements[0].StartLocation;
        }
        public Location EndLocation {
            get => Elements[^1].EndLocation;
        }

        public int StartTime {
            get => Elements[0].StartTime;
        }
        public int EndTime {
            get => Elements[^1].EndTime;
        }

        public int Duration {
            get => EndTime - StartTime;
        }
        public Descriptor Descriptor {
            get => new(StartLocation, StartTime, EndLocation, EndTime);
        }

        public static Block FromDescriptor(Location startLocation, int startTime, Location endLocation, int endTime) {
            Block b = new();
            b.Elements.Add(new BlockElement() {
                StartLocation = startLocation,
                StartTime = startTime,
                EndLocation = endLocation,
                EndTime = endTime,
            });
            return b;
        }

        public static List<Block> FromVehicleTask(VehicleTask vt) {
            // Transform into list of block elements
            List<BlockElement> elements = vt.Elements.SelectMany(e => BlockElement.FromVE(e)).ToList();

            // Iterate through elements in order to actually determine blocks
            List<Block> blocks = [new Block()];
            for (int i = 0; i < elements.Count; i++) {
                BlockElement element = elements[i];

                if (
                    IDLE_TYPES.Contains(element.Type) // vehicle is idle
                    && element.StartLocation.HandoverAllowed   // handover at this location is allowed
                    && Math.Max(element.StartLocation.SignOffTime, element.StartLocation.SignOnTime) <= element.EndTime - element.StartTime // idle time is greater than signon/off time
                ) {
                    // Start new block; skip this element as we dont need idle time in block
                    blocks.Add(new());
                    continue;
                }

                Block additionTarget = blocks[^1];
                additionTarget.Elements.Add(element);
            }

            var finalizedBlocks = blocks
                .Where(b => b.Elements.Count > 0 && b.Elements[^1].EndTime - b.Elements[0].StartTime > 0)
                .ToList();
            vt.BlockDescriptorCover = finalizedBlocks.Select(x => x.Descriptor).ToList();
            return finalizedBlocks;
        }

        public static List<(Block block, int count)> FromVehicleTasks(List<VehicleTask> vts) {
            List<Block> selectedBlocks = vts.SelectMany(t => Block.FromVehicleTask(t)).ToList();
            Dictionary<Descriptor, (Block firstRef, int count)> blockCounts = new();
            foreach (Block block in selectedBlocks) {
                Descriptor descriptor = block.Descriptor;
                blockCounts.TryAdd(descriptor, (block, 0));
                var curr = blockCounts[descriptor];
                blockCounts[descriptor] = (curr.firstRef, curr.count + 1);
            }

            // reindex after all blocks descriptors are unique
            var blockCountVals = blockCounts.Values.ToList();
            for (int i = 0; i < blockCountVals.Count; i++) {
                blockCountVals[i].firstRef.Index = i;
            }
            return blockCountVals;
        }

        public override string ToString() {
            return $"Block {Index}";
        }
    }
}

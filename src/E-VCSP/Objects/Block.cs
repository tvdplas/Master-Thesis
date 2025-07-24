using E_VCSP.Objects.ParsedData;
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
                    StartLocation = vet.Trip.From,
                    EndLocation = vet.Trip.To,
                });
            }
            else if (ve is VEDeadhead ved) {
                // Edge case for self arc "deadhead" 
                if (ved.DeadheadTemplate.Duration > 0) {
                    elements.Add(new BEDeadhead() {
                        DeadheadTemplate = ved.DeadheadTemplate,
                        EndTime = ved.EndTime,
                        StartTime = ved.StartTime,
                        StartLocation = ved.DeadheadTemplate.From,
                        EndLocation = ved.DeadheadTemplate.To,
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
    /// Part of a vehicle task / crew schedule
    /// </summary>
    public class Block {
        static List<BlockElementType> IDLE_TYPES = [BlockElementType.Idle, BlockElementType.Charge];

        List<BlockElement> Elements = new();
        public int Index = -1;

        public Location StartLocation {
            get {
                return Elements[0].StartLocation;
            }
        }
        public Location EndLocation {
            get {
                return Elements[^1].EndLocation;
            }
        }

        public int StartTime {
            get {
                return Elements[0].StartTime;
            }
        }
        public int EndTime {
            get {
                return Elements[^1].EndTime;
            }
        }

        public string Descriptor {
            get {
                return $"{StartLocation.Index}#{StartTime}#{EndLocation.Index}#{EndTime}";
            }
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

            return blocks
                .Where(b => b.Elements.Count > 0)
                .ToList();
        }

        public override string ToString() {
            return $"Block {Index}";
        }
    }
}

using E_VCSP.Objects.ParsedData;
using System.Collections;
using System.Text.Json.Serialization;

namespace E_VCSP.Objects {
    public enum CrewDutyElementType {
        Block,
        Idle,
        Break,
        Travel,
        SignOnOff,
    }
    public class CrewDutyElement {
        [JsonInclude]
        public CrewDutyElementType Type;
        [JsonInclude]
        public int StartTime;
        [JsonInclude]
        public int EndTime;
        [JsonInclude]
        public Location StartLocation;
        [JsonInclude]
        public Location EndLocation;
    }

    public class CDEBlock : CrewDutyElement {
        [JsonInclude]
        public Block Block;
        public CDEBlock(Block b) {
            Type = CrewDutyElementType.Block;
            StartTime = b.StartTime;
            EndTime = b.EndTime;
            StartLocation = b.StartLocation;
            EndLocation = b.EndLocation;
            Block = b;
        }
    }

    public class CDEIdle : CrewDutyElement {
        public CDEIdle(int startTime, int endTime, Location startLocation, Location endLocation) {
            Type = CrewDutyElementType.Idle;
            StartTime = startTime;
            EndTime = endTime;
            StartLocation = startLocation;
            EndLocation = endLocation;
        }
    }

    public class CDEBreak : CrewDutyElement {
        public CDEBreak(int startTime, int endTime, Location startLocation, Location endLocation) {
            Type = CrewDutyElementType.Break;
            StartTime = startTime;
            EndTime = endTime;
            StartLocation = startLocation;
            EndLocation = endLocation;
        }
    }

    public class CDETravel : CrewDutyElement {
        public CDETravel(int startTime, int endTime, Location startLocation, Location endLocation) {
            Type = CrewDutyElementType.Travel;
            StartTime = startTime;
            EndTime = endTime;
            StartLocation = startLocation;
            EndLocation = endLocation;
        }
    }

    public class CDESignOnOff : CrewDutyElement {
        public CDESignOnOff(int startTime, int endTime, Location location) {
            if (!location.CrewHub) throw new InvalidOperationException("Heh");

            Type = CrewDutyElementType.SignOnOff;
            StartTime = startTime;
            EndTime = endTime;
            StartLocation = location;
            EndLocation = location;
        }
    }

    public enum DutyType {
        Early, // Ends before 16:30
        Day, // Ends between 16:30 and 18:15
        Late, // Begin >= 13:00, end at max 26:30
        Night, // Begin < 24:00, end >= 26:30, max 7 hours long
        Between, // Begin before 13, ends after 18:15. Max 10% overall
        Broken, // Begin after 5:30, end before 19:30, min 1.5 hours of rest inbetween, max 30% of overall
        Count,
        Single, // Only a single block; only used for initialization
    }

    public class CrewDuty {
        [JsonInclude]
        public DutyType Type;
        [JsonInclude]
        public List<CrewDutyElement> Elements;
        public List<int> BlockCover;
        [JsonInclude]
        public int Index = -1;

        public double Cost {
            get {
                double cost = Config.CR_SHIFT_COST; // base
                // Driven time
                cost += Elements.Sum(e => (e.EndTime - e.StartTime) / (60.0 * 60.0) * Config.CR_HOURLY_COST);
                // Special type
                if (Type == DutyType.Single) cost += Config.CR_SINGLE_SHIFT_COST;
                // Special type + remove largest idle
                if (Type == DutyType.Broken) {
                    var largestIdle = Elements
                        .Where(e => e.Type == CrewDutyElementType.Idle)
                        .OrderByDescending(e => e.EndTime - e.StartTime)
                        .FirstOrDefault();

                    if (largestIdle == null) {
                        //throw new InvalidDataException("Broken shift with no idle");
                    }
                    else {
                        cost -= (largestIdle.EndTime - largestIdle.StartTime) / (60.0 * 60.0) * Config.CR_HOURLY_COST; // remove largest idle
                    }

                    cost += Config.CR_BROKEN_SHIFT_COST;
                }

                return cost;
            }
        }

        public CrewDuty(List<CrewDutyElement> elements) {
            Elements = elements;
            BlockCover = [.. elements.Where(e => e.Type == CrewDutyElementType.Block).Select(e => ((CDEBlock)e).Block.Index)];
        }

        public BitArray ToBitArray(int blockCount) {
            BitArray ba = new(blockCount);
            for (int i = 0; i < BlockCover.Count; i++) ba[BlockCover[i]] = true;
            return ba;
        }

        public override string ToString() {
            string type = "";
            switch (Type) {
                case DutyType.Early: type = "Early"; break;
                case DutyType.Day: type = "Day"; break;
                case DutyType.Late: type = "Late"; break;
                case DutyType.Broken: type = "Broken"; break;
                case DutyType.Between: type = "Between"; break;
                case DutyType.Night: type = "Night"; break;
                case DutyType.Single: type = "Single"; break;
            }

            return $"[{Index}] {type} duty";
        }

        public int Duration => Elements[^1].EndTime - Elements[0].StartTime;
    }
}


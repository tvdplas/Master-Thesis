using E_VCSP.Objects.ParsedData;
using System.Collections.Immutable;

namespace E_VCSP.Utils {
    public record struct DescriptorHalf {
        public int LocationIndex;
        public int Time;

        public DescriptorHalf(Location location, int time) {
            LocationIndex = location.Index;
            Time = time;
        }

        public DescriptorHalf(int locationIndex, int time) {
            LocationIndex = locationIndex;
            Time = time;
        }

        public Descriptor AddEnd(Location endLocation, int endTime) {
            return new Descriptor(LocationIndex, Time, endLocation.Index, endTime);
        }
    }

    public record struct Descriptor {
        public int StartLocationIndex;
        public int StartTime;
        public int EndLocationIndex;
        public int EndTime;

        public Descriptor(Location l1, int t1, Location l2, int t2) {
            StartLocationIndex = l1.Index;
            StartTime = t1;
            EndLocationIndex = l2.Index;
            EndTime = t2;
        }

        internal Descriptor(int l1, int t1, int l2, int t2) {
            StartLocationIndex = l1;
            StartTime = t1;
            EndLocationIndex = l2;
            EndTime = t2;
        }

        public Descriptor(string s) {
            var parts = s.Split('#').Select(int.Parse).ToImmutableArray();
            StartLocationIndex = parts[0];
            StartTime = parts[1];
            EndLocationIndex = parts[2];
            EndTime = parts[3];
        }

        public DescriptorHalf GetStart() => new DescriptorHalf(StartLocationIndex, StartTime);
        public DescriptorHalf GetEnd() => new DescriptorHalf(EndLocationIndex, EndTime);

        public override string ToString() {
            return $"{StartLocationIndex}#{StartTime}#{EndLocationIndex}#{EndTime}";
        }
    }
}

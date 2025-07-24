using System.Collections;

namespace E_VCSP.Utils {
    public class BitArrayComparer : IEqualityComparer<BitArray> {
        public bool Equals(BitArray? x, BitArray? y) {
            if (x == null && y == null) return true;
            if (x == null || y == null) return false;

            for (int i = 0; i < Math.Min(x.Length, y.Length); i++)
                if (x[i] != y[i]) return false;

            // Two bitarrays are considered the same if the length is different but the bits inside 
            // are the same
            if (x.Length != y.Length) {
                for (int i = Math.Min(x.Length, y.Length); i < Math.Max(x.Length, y.Length); i++) {
                    if (i < x.Length && x[i]) return false;
                    if (i < y.Length && y[i]) return false;
                }
            }
            return true;
        }

        public int GetHashCode(BitArray obj) {
            int[] ints = new int[(obj.Length + 31) / 32];
            obj.CopyTo(ints, 0);
            int hash = 17;
            foreach (int i in ints)
                hash = hash * 31 + i;
            return hash;
        }
    }
}

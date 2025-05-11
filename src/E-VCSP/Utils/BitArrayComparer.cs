using System.Collections;

namespace E_VCSP.Utils
{
    internal class BitArrayComparer : IEqualityComparer<BitArray>
    {
        public bool Equals(BitArray? x, BitArray? y)
        {
            if (x == null && y == null) return true;
            if (x == null || y == null) return false;

            if (x.Length != y.Length) return false;
            for (int i = 0; i < x.Length; i++)
                if (x[i] != y[i]) return false;
            return true;
        }

        public int GetHashCode(BitArray obj)
        {
            int[] ints = new int[(obj.Length + 31) / 32];
            obj.CopyTo(ints, 0);
            int hash = 17;
            foreach (int i in ints)
                hash = hash * 31 + i;
            return hash;
        }
    }
}

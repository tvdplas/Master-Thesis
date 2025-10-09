using System.Collections;

namespace E_VCSP.Utils {
    public class BitArrayIntComparer : IEqualityComparer<(BitArray, int)> {
        public bool Equals((BitArray, int) a, (BitArray, int) b) {
            return BitArrayComparer.IsEqual(a.Item1, b.Item1) && a.Item2 == b.Item2;
        }

        public int GetHashCode((BitArray, int) b) {
            unchecked {
                int h1 = BitArrayComparer.Hash(b.Item1);
                return (h1 * 397) ^ b.Item2;
            }
        }
    }

    public class BitArrayTupleComparer : IEqualityComparer<(BitArray, BitArray)> {
        public bool Equals((BitArray, BitArray) a, (BitArray, BitArray) b) {
            return BitArrayComparer.IsEqual(a.Item1, b.Item1) && BitArrayComparer.IsEqual(a.Item2, b.Item2);
        }

        public int GetHashCode((BitArray, BitArray) b) {
            unchecked {
                int h1 = BitArrayComparer.Hash(b.Item1);
                int h2 = BitArrayComparer.Hash(b.Item2);
                return (h1 * 397) ^ h2;
            }
        }
    }

    public class BitArrayComparer : IEqualityComparer<BitArray> {
        public bool Equals(BitArray? x, BitArray? y) => IsEqual(x, y);
        public int GetHashCode(BitArray b) => Hash(b);

        public static bool IsEqual(BitArray? x, BitArray? y) {
            if (x == null && y == null) return true;
            if (x == null || y == null) return false;
            int l = UsedLength(x);
            if (l != UsedLength(y)) return false;

            for (int i = 0; i < l; i++) {
                if (x[i] != y[i]) return false;
            }
            return true;
        }

        public static int UsedLength(BitArray b) {
            for (int i = b.Length - 1; i >= 0; i--)
                if (b[i]) return i + 1;
            return 0;
        }

        public static int Hash(BitArray b) {
            int len = UsedLength(b);
            unchecked {
                int hash = 17;
                for (int i = 0; i < len; i++)
                    hash = hash * 31 + (b[i] ? 1 : 0);
                return hash;
            }
        }
    }
}

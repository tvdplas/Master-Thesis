using E_VCSP.Objects.ParsedData;
using System.Runtime.CompilerServices;

namespace E_VCSP.Utils {
    internal static class Descriptor {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static string Create(Location location, int time) {
            return $"{location.Index}#{time}";
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static string Create(Location l1, int t1, Location l2, int t2) {
            return $"{l1.Index}#{t1}#{l2.Index}#{t2}";
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static string GetStart(string descriptor) => String.Join("#", descriptor.Split("#").Take(2));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static string GetEnd(string descriptor) => String.Join("#", descriptor.Split("#").Skip(2).Take(2));
    }
}

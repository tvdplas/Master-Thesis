namespace E_VCSP.Formatting
{
    internal static class Distance
    {
        internal static string KM(int meters, int decimals = 1)
        {
            return $"{Math.Round(meters / 1000.0, decimals)} km";
        }
    }
}

namespace E_VCSP.Objects
{
    /// <summary>
    /// Part of a piecewise linear approximation.
    /// </summary>
    internal class PiecewiseLinearPart
    {
        /// <summary>
        /// Valid X range in [start, end)
        /// </summary>
        internal double RangeStart;
        /// <summary>
        /// Valid X range in [start, end)
        /// </summary>
        internal double RangeEnd;
        /// <summary>
        /// Constant a in y = ax + b
        /// </summary>
        internal double A;
        /// <summary>
        /// Constant b in y = ax + b
        /// </summary>
        internal double B;
    }
    internal class PiecewiseLinear
    {

    }
}

using System.Runtime.CompilerServices;

namespace E_VCSP.Objects
{
    internal class ChargeResult
    {
        internal double SoCGained;
        internal double TimeUsed;
        internal double Cost;
    }

    /// <summary>
    /// Describes a piecewise linear charging curve.
    /// </summary>
    internal class ChargingCurve
    {
        private class CurvePiece
        {
            internal int MinSoC;
            internal int MaxSoC;
            internal double Rate;
        }

        // Sorted list of curve pieces
        private List<CurvePiece> Pieces;
        public double CostPerPercentage;

        public ChargingCurve(List<(int SoC, double rate)> rates, double totalCapacity)
        {
            if (rates.Count == 0)
            {
                throw new InvalidDataException("No charging rates provided");
            }

            Pieces = [];
            int prevSoC = 0;

            for (int i = 0; i < rates.Count; i++)
            {
                if (rates[i].rate < 0)
                {
                    throw new InvalidDataException("Provided charging rate was negative");
                }
                if (prevSoC >= rates[i].SoC)
                {
                    throw new InvalidDataException("Given list of rates don't describe a charging curve. SoC cannot decrease or stay the same.");
                }

                Pieces.Add(new()
                {
                    MinSoC = prevSoC,
                    MaxSoC = rates[i].SoC,
                    Rate = rates[i].rate,
                });
                prevSoC = rates[i].SoC;
            }

            // Charging cost per percentage gained 
            CostPerPercentage = Config.KWH_COST * totalCapacity / 100;
        }

        /// <summary>
        /// Get piece corresponding to current SoC
        /// </summary>
        /// <param name="currSoC">Current SoC</param>
        /// <param name="expand">Allow expansion</param>
        /// <returns>Piece if one exists</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        CurvePiece? getPiece(double currSoC, bool expand)
        {
            CurvePiece? p = expand ? Pieces[^1] : null;
            for (int i = 0; i < Pieces.Count; i++)
            {
                if (currSoC <= Pieces[i].MaxSoC)
                {
                    p = Pieces[i];
                    break;
                }
            }

            return p;
        }

        /// <summary>
        /// Returns the maximum amount of percentage gained, along with the time used in order to get this percentage.
        /// Additionally, returns the time used in order to achieve this SoC; will only be lower than time provided if battery is full
        /// </summary>
        /// <param name="startSoC">SoC at start of charging operation</param>
        /// <param name="time">Time allowed for use in the operation</param>
        /// <param name="expand">Expand charging curve to allow for &lt; 0 and &gt; 100% by taking closes linepiece.</param>
        /// <returns>Tuple with maxSoCGained, timetaken and total cost if charging actions is performed fully</returns>
        public ChargeResult MaxChargeGained(double startSoC, int time, bool expand)
        {
            int timeRemaining = time;
            double currSoC = startSoC;

            if (!expand && startSoC < 0) throw new InvalidDataException("Cannot start charging with SoC < 0. Did you forget to enable expansion?");

            while (timeRemaining > 0 && (expand || currSoC < 100))
            {
                // Get the part of the charging curve which is currently applicable.
                CurvePiece? p = getPiece(currSoC, expand);
                if (p == null) throw new InvalidDataException("Charging curve is not defined for the current SoC");

                // Determine amount of time to get from current SoC to the piece max SoC
                double maxSoCInPiece = expand && p.MaxSoC == 100 ? 1_000_000 : p.MaxSoC;
                int timeToMax = (int)Math.Ceiling((maxSoCInPiece - currSoC) / (1.0 * p.Rate));
                int usableTime = Math.Min(timeRemaining, timeToMax);
                double gainableSoC = usableTime * p.Rate;
                currSoC += gainableSoC;
                timeRemaining -= usableTime;
            }

            return new ChargeResult()
            {
                SoCGained = currSoC - startSoC,
                TimeUsed = time - timeRemaining,
                Cost = CostPerPercentage * (currSoC - startSoC),
            };
        }

        /// <summary>
        /// Get the charging costs associated with charging to a certain SoC
        /// </summary>
        /// <param name="startSoC">SoC that vehicle starts at</param>
        /// <param name="targetSoC">Target SoC</param>
        /// <returns>Charge result representing the requested charge</returns>
        public ChargeResult ChargeCosts(double startSoC, double targetSoC, bool expand)
        {
            double currSoC = startSoC;
            int currTime = 0;

            if (!expand && (startSoC < 0 || startSoC > 100 || targetSoC < 0 || targetSoC > 100))
                throw new InvalidDataException("SoC out of bounds. Did you forget to enable expansion?");

            while (currSoC < targetSoC)
            {
                // First piece that fits; default to last (slowest)
                CurvePiece? p = getPiece(currSoC, expand);
                if (p == null) throw new InvalidDataException("No piece found");

                double maxSoCInPiece = expand && currSoC >= 100
                    ? 1_000_000
                    : p.MaxSoC;

                // Charge until end of piece or until desired SoC is reached
                double SoCChargedInPiece = Math.Min(targetSoC - currSoC, maxSoCInPiece - currSoC);
                int timeUsed = (int)Math.Ceiling(SoCChargedInPiece / p.Rate);
                currSoC += SoCChargedInPiece;
                currTime += timeUsed;
            }

            return new ChargeResult()
            {
                SoCGained = targetSoC - startSoC,
                TimeUsed = currTime,
                Cost = CostPerPercentage * (targetSoC - startSoC),
            };
        }
    }
}

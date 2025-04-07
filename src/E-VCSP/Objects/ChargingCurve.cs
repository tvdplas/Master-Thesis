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
        /// Returns the maximum amount of percentage gained, along with the time used in order to get this percentage.
        /// Additionally, returns the time used in order to achieve this SoC; will only be lower than time provided if battery is full
        /// </summary>
        /// <param name="startSoC">SoC at start of charging operation</param>
        /// <param name="time">Time allowed for use in the operation</param>
        /// <returns>Tuple with maxSoCGained, timetaken and total cost if charging actions is performed fully</returns>
        public ChargeResult MaxChargeGained(double startSoC, int time)
        {
            int timeRemaining = time;
            double currSoC = startSoC;
            while (timeRemaining > 0 && currSoC < 100)
            {
                // Get the part of the charging curve which is currently applicable.
                CurvePiece? p = Pieces.Find(piece => piece.MinSoC <= currSoC && piece.MaxSoC > currSoC);
                if (p == null) throw new InvalidDataException("Charging curve is not defined for the current SoC");

                // Determine amount of time to get from current SoC to the piece max SoC
                int timeToMax = (int)Math.Ceiling((p.MaxSoC - currSoC) / (1.0 * p.Rate));
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
        public ChargeResult ChargeCosts(double startSoC, double targetSoC)
        {
            double currSoC = startSoC;
            int currTime = 0;

            while (currSoC < targetSoC)
            {
                CurvePiece? p = null;
                for (int i = 0; i < Pieces.Count; i++) if (Pieces[i].MinSoC <= currSoC && Pieces[i].MaxSoC >= currSoC)
                    {
                        p = Pieces[i];
                        break;
                    }
                if (p == null) throw new InvalidDataException("Charging curve is not defined for the current SoC");

                // Charge until end of piece or until desired SoC is reached
                double SoCChargedInPiece = Math.Min(targetSoC - currSoC, p.MaxSoC - currSoC);
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

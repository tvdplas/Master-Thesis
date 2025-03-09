namespace E_VCSP.Objects
{
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

        internal class ChargeResult
        {
            internal double SoCGained;
            internal double TimeUsed;
            internal double Cost;
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
        public ChargeResult ChargeGained(double startSoC, int time)
        {
            int timeRemaining = time;
            double currSoC = startSoC;
            while (timeRemaining > 0 && currSoC < 100)
            {
                // Get the part of the charging curve which is currently applicable.
                CurvePiece? p = Pieces.Find(piece => piece.MinSoC <= currSoC && piece.MaxSoC > currSoC);
                if (p == null)
                {
                    throw new InvalidDataException("Charging curve is not defined for the current SoC");
                }

                // Determine amount of time to get from current SoC to the piece max SoC
                int timeToMax = (int)Math.Ceiling(p.MaxSoC - (currSoC / (1.0 * p.Rate)));
                int usableTime = Math.Min(timeRemaining, timeToMax);
                int gainableSoC = (int)Math.Floor(usableTime * p.Rate);
                currSoC += gainableSoC;
                timeRemaining -= usableTime;
            }

            // TODO: de SoC kan nu wel non-linear, maar de kosten doen dat niet; 
            // aanpassing is nodig waarbij je na het berekenen van gebruik opnieuw de kosten bepaalt.
            return new ChargeResult()
            {
                SoCGained = currSoC - startSoC,
                TimeUsed = -(time - timeRemaining),
                Cost = CostPerPercentage * (currSoC - startSoC),
            };
        }
    }
}

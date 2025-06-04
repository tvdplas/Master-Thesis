using Microsoft.Msagl.Drawing;

namespace E_VCSP.Solver
{
    public abstract class Solver
    {
        /// <summary>
        /// Solve loaded instance
        /// </summary>
        /// <returns><c>true</c> if solving was successful; <c>false</c> otherwise</returns>
        public abstract bool Solve(CancellationToken cancellationToken);

        /// <summary>
        /// Generate solution graph
        /// </summary>
        /// <returns>Graph</returns>
        public abstract Graph GenerateSolutionGraph(bool blockView);
    }
}

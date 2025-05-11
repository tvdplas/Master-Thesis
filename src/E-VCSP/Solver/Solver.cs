using Microsoft.Msagl.Drawing;

namespace E_VCSP.Solver
{
    internal abstract class Solver
    {
        /// <summary>
        /// Solve loaded instance
        /// </summary>
        /// <returns><c>true</c> if solving was successful; <c>false</c> otherwise</returns>
        internal abstract bool Solve(CancellationToken cancellationToken);

        /// <summary>
        /// Generate solution graph
        /// </summary>
        /// <returns>Graph</returns>
        internal abstract Graph GenerateSolutionGraph();
    }
}

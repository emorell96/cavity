using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace PoundDreverHallTracking.Generators
{
    public interface ITransferFunction
    {
        public Complex CalculateTransferValue(double omega, double time);
    }
}

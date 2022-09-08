using MathNet.Numerics;
using PoundDreverHallTracking.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace PoundDreverHallTracking.Generators
{
    public class PoundDreverHallSignalGenerator : ITraceGenerator, ITransferFunction
    {
        public double StartX { get; set; }
        public double EndX { get; set; }
        public int Steps { get; set; }

        public double PowerCarrier { get; set; }
        public double PowerSideband { get; set; }

        public double CavityFsr { get; set; }
        public double Reflection { get; set; }
        public double ModulationFrequency { get; set; }


        public Complex CalculateTransferValue(double omega, double time = 0)
        {
            var phase = omega / CavityFsr;

            var numerator = Reflection * (Complex.FromPolarCoordinates(1, phase) - 1);
            var denominator = 1 - Math.Pow(Reflection, 2) * Complex.FromPolarCoordinates(1, phase);

            return numerator / denominator;
        }

        public double CalculateError(double omega)
        {
            return 2 * Math.Sqrt(PowerCarrier * PowerSideband) * (CalculateTransferValue(omega) * Complex.Conjugate(CalculateTransferValue(omega + ModulationFrequency)) - Complex.Conjugate(CalculateTransferValue(omega)) * CalculateTransferValue(omega - ModulationFrequency)).Imaginary;
        }

        public List<Point> GenerateTrace()
        {
            var output = new List<Point>();
            var range = Generate.LinearSpaced(Steps, StartX, EndX);
            foreach(var value in range)
            {
                output.Add(new Point(value, CalculateError(value)));
            }
            return output;
        }
    }
}

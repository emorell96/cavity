using PoundDreverHallTracking.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PoundDreverHallTracking.Generators
{
    public interface ITraceGenerator
    {
        public Trace GenerateTrace();
    }
}

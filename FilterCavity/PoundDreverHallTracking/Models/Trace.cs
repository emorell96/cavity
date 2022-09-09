using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PoundDreverHallTracking.Models
{
    public record Trace(Point Maxima, Point Minima, List<Point> Points)
    {
    }
}

using PoundDreverHallTracking.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PoundDreverHallTracking.Tracking
{
    public class PoundDreverHallSignal
    {
        public long MinimumVoltage { get; private set; }
        public long MaximumVoltage { get; private set; }
        
        public Point? Maxima { get; set; }
        public Point? Minima { get; set; }

        public bool IsSlopeRising
        {
            get
            {
                if(Maxima is not null && Minima is not null)
                {
                    return (Maxima.Y - Minima.Y) / (Maxima.X - Minima.X) > 0;
                }
                throw new InvalidDataException();
            }
        }


        public void AddPoint(Point point)
        {
            // sets the points if they are null (i.e. we just started).
            Maxima ??= point;
            Minima ??= point;

            if(Maxima.Y < point.Y)
            {
                Maxima = point;
            }

            if(Minima.Y > point.Y)
            {
                Minima = point;
            }
        } 
    }
}

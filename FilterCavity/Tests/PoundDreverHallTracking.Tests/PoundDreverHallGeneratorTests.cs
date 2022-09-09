using CsvHelper;
using FluentAssertions;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using PoundDreverHallTracking.Generators;
using PoundDreverHallTracking.Tracking;
using System.Globalization;

namespace PoundDreverHallTracking.Tests
{
    [TestClass]
    public class PoundDreverHallGeneratorTests
    {
        [DataRow(false)]
        [DataRow(true)]
        [TestMethod]
        public void GenerateTrace(bool invert)
        {
            var generator = new PoundDreverHallSignalGenerator
            {
                PowerSideband = 0.5,
                PowerCarrier = 1,
                CavityFsr = 10000,
                ModulationFrequency = 2000000,
                Reflection = 0.95,
                Steps = 400,
                StartX =-50000,
                EndX = 50000,
                Invert = invert,

            };
            var trace = generator.GenerateTrace();
            
            Assert.IsNotNull(trace);
            // save the trace
            using (var writer = new StreamWriter("pdhTrace.csv"))
            using (var csv = new CsvWriter(writer, CultureInfo.InvariantCulture))
            {

                //csv.WriteField("Minima");
                //csv.WriteRecord(trace.Minima);
                //csv.NextRecord();
                //csv.WriteField("Maxima");
                //csv.WriteRecord(trace.Maxima);
                //csv.NextRecord();
                csv.WriteRecords(trace.Points);

            }

            var signal = new PoundDreverHallSignal();

            foreach(var point in trace.Points)
            {
                signal.AddPoint(point);
            }

            signal.Minima.Should().BeEquivalentTo(trace.Minima);
            signal.Maxima.Should().BeEquivalentTo(trace.Maxima);
            signal.IsSlopeRising.Should().Be(invert);
        }
    }
}
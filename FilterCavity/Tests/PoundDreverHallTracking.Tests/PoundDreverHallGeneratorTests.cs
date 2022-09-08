using CsvHelper;
using PoundDreverHallTracking.Generators;
using System.Globalization;

namespace PoundDreverHallTracking.Tests
{
    [TestClass]
    public class PoundDreverHallGeneratorTests
    {

        [TestMethod]
        public void GenerateTrace()
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
                EndX = 50000

            };
            var trace = generator.GenerateTrace();
            
            Assert.IsNotNull(trace);
            // save the trace
            using (var writer = new StreamWriter("pdhTrace.csv"))
            using (var csv = new CsvWriter(writer, CultureInfo.InvariantCulture))
            {
                csv.WriteRecords(trace);
            }
        }
    }
}
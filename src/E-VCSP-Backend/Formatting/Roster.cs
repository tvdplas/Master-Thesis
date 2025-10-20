using E_VCSP.Formatting;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace E_VCSP_Backend.Formatting {
    public class Roster {
        [JsonInclude]
        public int Type = -1; // 0 = vehicle, 1 = block, 2 = crew;
        [JsonInclude]
        public string Comment = "";
        [JsonInclude]
        public List<List<RosterNode>> RosterNodes = [];

        public Roster() { }

        public Roster(int type, string comment, List<List<RosterNode>> rosterNodes) {
            Type = type;
            Comment = comment;
            RosterNodes = rosterNodes;
        }

        public void Dump(string targetPathNoType) {
            File.WriteAllText(targetPathNoType + $"-{Type}.json", JsonSerializer.Serialize(this));
        }
    }

    public class RosterNode {
        [JsonInclude]
        public required int StartTime;
        [JsonInclude]
        public required int EndTime;
        [JsonInclude]
        public required string Content;
        [JsonInclude]
        public required string ColorHex;

        public string DisplayContent {
            get {
                return $"{Content}\n{Time.HHMMSS(StartTime)}-{Time.HHMMSS(EndTime)}";
            }
        }
    }
}

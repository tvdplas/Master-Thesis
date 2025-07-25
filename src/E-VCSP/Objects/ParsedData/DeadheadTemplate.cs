﻿using System.Text.Json.Serialization;

namespace E_VCSP.Objects.ParsedData {
    /// <summary>
    /// Non-time bound deadhead.
    /// </summary>
    public class DeadheadTemplate {
        public Location From;
        public Location To;
        public int Duration;
        public int Distance;
        [JsonInclude]
        public required string Id;

        public override string ToString() {
            return Id;
        }
        public string ToLongString(bool showInfo) {
            return $"{From} -> {To}" + (showInfo ? $"\n{Formatting.Time.HHMMSS(Duration)} ({Formatting.Distance.KM(Distance)})" : "");
        }
    }
}

using E_VCSP.Objects.ParsedData;

namespace E_VCSP.Parsing
{
    public abstract class ParserBase<T>
    {
        public List<(string, string)>? attributeNameMapping;
        public string? filename;

        /// <summary>
        /// Match attributes to column indexes in the input
        /// </summary>
        /// <param name="header">Header columns</param>
        /// <returns>Dictionary d such that d[attr_name] = col_index</returns>
        /// <exception cref="ArgumentNullException">No attribute to column name mapping provided</exception>
        /// <exception cref="Exception">Invalid mapping provided</exception>
        public Dictionary<string, int> GetAttributeIndexMapping(List<string> header)
        {
            if (attributeNameMapping == null)
            {
                throw new ArgumentNullException("Invalid instantiation of ParserBase.");
            }

            Dictionary<string, int> attributeIndexMapping = new Dictionary<string, int>();
            foreach (var kv in attributeNameMapping)
            {
                int index = header.FindIndex(col => col.Contains(kv.Item2));
                if (index == -1) throw new Exception($"Could not find a column {kv.Item2} to load attribute {kv.Item1}");
                attributeIndexMapping[kv.Item1] = index;
            }

            return attributeIndexMapping;
        }

        /// <summary>
        /// Parse a time in format ss, mm:ss, or hh:mm:ss.
        /// </summary>
        /// <param name="s">Time string</param>
        /// <returns>Amount of seconds representing <paramref name="s"/></returns>
        /// <exception cref="ArgumentException">Input not in any of the listed formats.</exception>
        public int ParseTime(string s)
        {
            List<int> units = s.Split(':').Select(int.Parse).ToList();
            if (units.Count == 1)
            {
                return units[0];
            }
            else if (units.Count == 2)
            {
                return (units[0] * 60 * 60) + (units[1] * 60);
            }
            else if (units.Count == 3)
            {
                return (units[0] * 60 * 60) + (units[1] * 60) + units[2];
            }

            throw new ArgumentException($"Provided time string {s} was not in hh:mm or hh:mm:ss format");
        }

        /// <summary>
        /// Returns a reference to a location with the given id; If one is already present, return that. 
        /// Otherwise, create the location and add it.
        /// </summary>
        /// <param name="id">Id of location</param>
        /// <param name="locations">Active list of locations</param>
        /// <returns>Location with id=<paramref name="id"/></returns>
        public static Location GetOrCreateLocation(string id, List<Location> locations)
        {
            Location val;
            if (locations.Find(x => x.Id == id) is Location toLoc)
            {
                val = toLoc;
            }
            else
            {
                val = new Location() { Id = id, Index = locations.Count };
                locations.Add(val);
            }
            return val;
        }

        /// <summary>
        /// Parses a list of <typeparamref name="T"/>
        /// </summary>
        /// <param name="path">Folder containing the to be parsed file</param>
        /// <param name="locations">List of currently known locations; Note that parsing may introduce new locations if the parsed objects contains any that were unknown before.</param>
        /// <returns>List of <typeparamref name="T"/></returns>
        /// <exception cref="ArgumentNullException"></exception>
        public List<T> Parse(string path, List<Location> locations)
        {
            if (filename == null)
            {
                throw new ArgumentNullException("Invalid instantiation of ParserBase.");
            }

            List<List<string>> file = File.ReadAllLines($"{path}\\{filename}")
                .Select(line => line.Split(";").ToList()).ToList();
            var attributeIndexMapping = GetAttributeIndexMapping(file[0]);

            List<T> res = new();
            for (int i = 1; i < file.Count; i++)
            {
                res.Add(ParseSingle(
                    i - 1,
                    file[0],
                    file[i],
                    attributeIndexMapping,
                    locations
                ));
            }
            return res;
        }

        /// <summary>
        /// Parses a single <typeparamref name="T"/>
        /// </summary>
        /// <param name="index">Line index being parsed</param>
        /// <param name="line">Line parts</param>
        /// <param name="attributeIndexMapping">Attribute name to line part index mapping.</param>
        /// <param name="locations">Active locations</param>
        /// <returns><typeparamref name="T"/> parsed from <paramref name="line"/>. May introduce new entries in <paramref name="locations"/> if they were not yet known.</returns>
        public abstract T ParseSingle(
            int index,
            List<string> headers,
            List<string> line,
            Dictionary<string, int> attributeIndexMapping,
            List<Location> locations
        );
    }
}

namespace E_VCSP.Parsing
{
    internal abstract class ParserBase<T>
    {
        // <string1, string2>: 
        // string1: Attribute name
        // string2: Column name in dataset
        internal List<(string, string)>? attributeNameMapping;
        internal string? filename;
        internal Dictionary<string, int> getAttributeIndexMapping(List<string> header)
        {
            if (attributeNameMapping == null)
            {
                throw new ArgumentNullException("Invalid instantiation of ParserBase.");
            }

            Dictionary<string, int> attributeIndexMapping = new Dictionary<string, int>();
            foreach (var kv in attributeNameMapping)
            {
                int index = header.IndexOf(kv.Item2);
                if (index == -1) throw new Exception($"Could not find a column {kv.Item2} to load attribute {kv.Item1}");
                attributeIndexMapping[kv.Item1] = index;
            }

            return attributeIndexMapping;
        }

        internal int ParseTime(string s)
        {
            List<int> units = s.Split(':').Select(int.Parse).ToList();
            if (units.Count == 1)
            {
                return units[0];
            }
            else if (units.Count == 2)
            {
                return units[0] * 60 * 60 + units[1] * 60;
            }
            else if (units.Count == 3)
            {
                return units[0] * 60 * 60 + units[1] * 60 + units[2];
            }

            throw new ArgumentException($"Provided time string {s} was not in hh:mm or hh:mm:ss format");
        }

        internal List<T> Parse(string path)
        {
            if (filename == null)
            {
                throw new ArgumentNullException("Invalid instantiation of ParserBase.");
            }

            List<List<string>> file = File.ReadAllLines($"{path}\\{filename}")
                .Select(line => line.Split(";").ToList()).ToList();
            var attributeIndexMapping = getAttributeIndexMapping(file[0]);

            List<T> res = new();
            for (int i = 1; i < file.Count; i++)
            {
                res.Add(ParseSingle(i, file[i], attributeIndexMapping));
            }
            return res;
        }

        internal abstract T ParseSingle(int index, List<string> line, Dictionary<string, int> attributeIndexMapping);
    }
}

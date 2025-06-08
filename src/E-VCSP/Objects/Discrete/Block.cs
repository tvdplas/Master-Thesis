namespace E_VCSP.Objects.Discrete
{
    public enum BlockElementType
    {
        Idle,
        Trip,
        Deadhead,
        Charge,
    }
    public class BlockElement
    {
        public int StartTime;
        public int EndTime;
        public Location From;
        public Location To;
        public BlockElementType Type;

        public static List<BlockElement> FromVE(VehicleElement ve)
        {
            List<BlockElement> elements = new();

            if (ve is VEIdle vei)
            {
                elements.Add(new()
                {
                    Type = BlockElementType.Idle,
                    EndTime = ve.EndTime,
                    StartTime = ve.StartTime,
                    From = vei.StartLocation!,
                    To = vei.EndLocation!,
                });
            }
            else if (ve is VETrip vet)
            {
                elements.Add(new BETrip()
                {
                    Type = BlockElementType.Trip,
                    Trip = vet.Trip,
                    StartTime = vet.StartTime,
                    EndTime = vet.EndTime,
                    From = vet.Trip.From,
                    To = vet.Trip.To,
                });
            }
            else if (ve is VEDeadhead ved)
            {
                elements.Add(new BEDeadhead()
                {
                    DeadheadTemplate = ved.DeadheadTemplate,
                    EndTime = ved.EndTime,
                    StartTime = ved.StartTime,
                    From = ved.DeadheadTemplate.From,
                    To = ved.DeadheadTemplate.To,
                    Type = BlockElementType.Deadhead,
                });
            }
            else if (ve is VECharge vec)
            {
                elements.Add(new BlockElement()
                {
                    EndTime = vec.EndTime,
                    StartTime = vec.StartTime,
                    From = vec.StartLocation!,
                    To = vec.EndLocation!,
                    Type = BlockElementType.Charge,
                });
            }
            else
            {
                throw new InvalidDataException("Forgot to add VE case in conversion to BE");
            }

            return elements;
        }
    }

    public class BETrip : BlockElement
    {
        public required Trip Trip;
    }

    public class BEDeadhead : BlockElement
    {
        public required DeadheadTemplate DeadheadTemplate;
    }


    /// <summary>
    /// Part of a vehicle task / crew schedule
    /// </summary>
    public class Block
    {
        static List<BlockElementType> IDLE_TYPES = [BlockElementType.Idle, BlockElementType.Charge];

        List<BlockElement> Elements = new();

        public Location From
        {
            get
            {
                return Elements[0].From;
            }
        }
        public Location To
        {
            get
            {
                return Elements[^1].To;
            }
        }

        public int StartTime
        {
            get
            {
                return Elements[0].StartTime;
            }
        }
        public int EndTime
        {
            get
            {
                return Elements[^1].EndTime;
            }
        }

        public static List<Block> FromVehicleTask(VehicleTask vt)
        {
            // Transform into list of block elements
            List<BlockElement> elements = vt.Elements.SelectMany(e => BlockElement.FromVE(e)).ToList();

            // Iterate through elements in order to actually determine blocks
            List<Block> blocks = [new Block()];
            for (int i = 0; i < elements.Count; i++)
            {
                BlockElement element = elements[i];

                if (
                    IDLE_TYPES.Contains(element.Type) // vehicle is idle
                    && element.From.HandoverAllowed   // handover at this location is allowed
                    && Math.Max(element.From.SignOffTime, element.From.SignOnTime) <= (element.EndTime - element.StartTime) // idle time is greater than signon/off time
                )
                {
                    // Start new block; skip this element as we dont need idle time in block
                    blocks.Add(new());
                    continue;
                }

                Block additionTarget = blocks[^1];
                additionTarget.Elements.Add(element);
            }

            blocks = blocks.Where(b => b.Elements.Count > 0).ToList();

            return blocks;
        }
    }
}

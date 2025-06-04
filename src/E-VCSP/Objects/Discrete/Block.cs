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

            if (ve is VEDepot) return elements;
            else if (ve is VEIdle vei)
            {
                elements.Add(new()
                {
                    Type = BlockElementType.Idle,
                    EndTime = ve.EndTime,
                    StartTime = ve.StartTime,
                    From = vei.Location,
                    To = vei.Location,
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
                if (ved.SelectedAction == -1)
                {
                    // Normal deadhead
                    elements.Add(new BEDeadhead()
                    {
                        DeadheadTemplate = ved.Deadhead.DeadheadTemplate,
                        EndTime = ved.EndTime,
                        StartTime = ved.StartTime,
                        From = ved.Deadhead.DeadheadTemplate.From,
                        To = ved.Deadhead.DeadheadTemplate.To,
                        Type = BlockElementType.Deadhead,
                    });
                }
                else
                {
                    // Seperate into dh1 -> charge -> dh2
                    var action = ved.Deadhead.ChargingActions[ved.SelectedAction];

                    elements.Add(new BEDeadhead()
                    {
                        DeadheadTemplate = action.TemplateTo,
                        StartTime = ved.StartTime,
                        EndTime = ved.StartTime + action.DrivingTimeTo,
                        From = action.TemplateTo.From,
                        To = action.TemplateTo.To,
                        Type = BlockElementType.Deadhead,
                    });
                    elements.Add(new BlockElement()
                    {
                        EndTime = ved.StartTime + action.DrivingTimeTo + action.TimeAtLocation,
                        StartTime = ved.StartTime + action.DrivingTimeTo,
                        From = action.ChargeLocation,
                        To = action.ChargeLocation,
                        Type = BlockElementType.Charge,
                    });
                    elements.Add(new BEDeadhead()
                    {
                        DeadheadTemplate = action.TemplateFrom,
                        StartTime = ved.StartTime + action.DrivingTimeTo + action.TimeAtLocation,
                        EndTime = ved.EndTime,
                        From = action.TemplateFrom.From,
                        To = action.TemplateFrom.To,
                        Type = BlockElementType.Deadhead,
                    });
                }
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

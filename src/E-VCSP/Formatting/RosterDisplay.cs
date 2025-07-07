using E_VCSP.Formatting;
using System.Drawing.Drawing2D;
public class RosterNode
{
    public required int StartTime;
    public required int EndTime;
    public required string Content;
    public required Color Color;

    public string DisplayContent
    {
        get
        {
            return $"{Content}\n{Time.HHMMSS(StartTime)}-{Time.HHMMSS(EndTime)}";
        }
    }
}

public class RosterDisplay : Control
{
    private float _zoom = 1.0f;
    private float _horizontalZoom = 1.0f;
    private float _verticalZoom = 1.0f;
    private const float ZoomFactor = 1.1f;
    private const float MinZoom = 0.1f;
    private const float MaxZoom = 100000f;

    private const int rowHeight = 100;
    private const int rowMargin = 10;

    private float RowHeight => rowHeight * _verticalZoom;
    private float RowMargin => rowMargin * _verticalZoom;

    private const int borderWidth = 5;

    private List<Font> fontOptions = [
        new Font("Arial", 4),
        new Font("Arial", 8),
        new Font("Arial", 12),
    ];

    private Font clockFont = new Font("Arial", 24);

    private Point _panOffset = Point.Empty;
    private Point _mouseDownPoint;
    private Point _panStart;

    private List<List<RosterNode>> rosterNodes = [];

    private float rx(float x) => x * _horizontalZoom;

    public void ResetView(bool resetPan = true)
    {
        if (rosterNodes.Any() && resetPan)
            _panOffset = new Point(-rosterNodes.Min(x => x.Min(y => y.StartTime)), 0);

        this.Invalidate();
    }

    public void UpdateRosterNodes(List<List<RosterNode>> rosterNodes)
    {
        bool resetPan = !this.rosterNodes.Any() && rosterNodes.Any();
        this.rosterNodes = rosterNodes;

        ResetView(resetPan);
    }


    public RosterDisplay()
    {
        this.DoubleBuffered = true;
        this.Dock = DockStyle.Fill;

        this.MouseDown += RosterDisplay_MouseDown;
        this.MouseMove += RosterDisplay_MouseMove;
        this.MouseUp += RosterDisplay_MouseUp;
        this.MouseWheel += RosterDisplay_MouseWheel;
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        Graphics g = e.Graphics;
        g.SmoothingMode = SmoothingMode.HighSpeed;
        g.CompositingQuality = CompositingQuality.HighSpeed;
        g.Clear(Color.White);
        g.TranslateTransform(_panOffset.X, _panOffset.Y);
        g.ScaleTransform(_zoom, _zoom);

        if (!rosterNodes.Any())
        {
            g.DrawString("No data available", clockFont, Brushes.Black, new PointF(10, 10));
            return;
        }

        // Draw time lines at top
        int minTime = rosterNodes.Min(x => x.Min(y => y.StartTime));
        int maxTime = rosterNodes.Max(x => x.Max(y => y.EndTime));

        int startHour = minTime / 3600;
        int endHour = maxTime / 3600 + 1; // Klopt niet helemaal maar goed

        for (int i = startHour; i <= endHour; i++)
        {
            int x = i * 3600;
            g.DrawLine(new Pen(Color.DarkGray, 4), rx(x), -100, rx(x), rosterNodes.Count * (RowHeight + RowMargin));
            g.DrawLine(new Pen(Color.LightGray, 4), rx(x + 900), -100, rx(x + 900), rosterNodes.Count * (RowHeight + RowMargin));
            g.DrawLine(new Pen(Color.LightGray, 4), rx(x + 1800), -100, rx(x + 1800), rosterNodes.Count * (RowHeight + RowMargin));
            g.DrawLine(new Pen(Color.LightGray, 4), rx(x + 2700), -100, rx(x + 2700), rosterNodes.Count * (RowHeight + RowMargin));
            g.DrawString(Time.HHMMSS(x), clockFont, Brushes.Black, new PointF(rx(x + 10), -90));
        }


        // Sort nodes by color, draw all rectangles in single call, add text afterwards
        Dictionary<Color, List<RectangleF>> bufferedRectangles = new();
        List<(PointF p, string s, Font f)> bufferedText = new();

        for (int row = 0; row < rosterNodes.Count; row++)
        {
            foreach (var node in rosterNodes[row])
            {
                float width = rx(node.EndTime - node.StartTime);
                if (width <= 1) continue;

                float top = row * (RowHeight + RowMargin);
                float left = rx(node.StartTime);
                RectangleF rf = new RectangleF(left, top, width, RowHeight);
                if (!g.IsVisible(rf)) continue;

                if (bufferedRectangles.ContainsKey(node.Color)) bufferedRectangles[node.Color].Add(rf);
                else bufferedRectangles[node.Color] = [rf];

                if (_zoom < 0.5f) continue; // Don't draw text if zoomed out too far
                for (int i = fontOptions.Count - 1; i >= 0; i--)
                {
                    var measuredString = g.MeasureString(node.DisplayContent, fontOptions[i]);
                    if (measuredString.Width > rx(node.EndTime - node.StartTime) - borderWidth * 4 ||
                        measuredString.Height > RowHeight - borderWidth * 4) continue;

                    bufferedText.Add((
                        new PointF(left + (width - measuredString.Width) / 2f, top + (RowHeight - measuredString.Height) / 2f),
                        node.DisplayContent,
                        fontOptions[i]
                    ));
                    break;
                }

            }

            foreach (var kvp in bufferedRectangles)
            {
                g.FillRectangles(new SolidBrush(kvp.Key), kvp.Value.ToArray());
                g.DrawRectangles(new Pen(Color.Black, borderWidth), kvp.Value.ToArray());
            }
            foreach (var (p, s, f) in bufferedText)
            {
                g.DrawString(s, f, Brushes.Black, p);
            }
        }

        base.OnPaint(e);
    }

    private void RosterDisplay_MouseDown(object sender, MouseEventArgs e)
    {
        if (e.Button == MouseButtons.Left)
        {
            _mouseDownPoint = e.Location;
            _panStart = _panOffset;
            this.Cursor = Cursors.Hand;
        }
    }

    private void RosterDisplay_MouseMove(object sender, MouseEventArgs e)
    {
        if (e.Button == MouseButtons.Left)
        {
            int dx = e.X - _mouseDownPoint.X;
            int dy = e.Y - _mouseDownPoint.Y;
            _panOffset = new Point(_panStart.X + dx, _panStart.Y + dy);
            this.Invalidate();
        }
    }

    private void RosterDisplay_MouseUp(object sender, MouseEventArgs e)
    {
        if (e.Button == MouseButtons.Left)
        {
            this.Cursor = Cursors.Default;
        }
    }

    private void RosterDisplay_MouseWheel(object sender, MouseEventArgs e)
    {
        if ((Control.ModifierKeys & Keys.Shift) == Keys.Shift)
        {
            float oldHorizontalZoom = _horizontalZoom;
            if (e.Delta > 0) _horizontalZoom *= ZoomFactor;
            else _horizontalZoom /= ZoomFactor;

            // Rescale pan offset to maintain position
            _panOffset = new Point(
                (int)(_panOffset.X * _horizontalZoom / oldHorizontalZoom),
                _panOffset.Y
            );
        }
        else if ((Control.ModifierKeys & Keys.Control) == Keys.Control)
        {
            float oldVerticalZoom = _verticalZoom;
            if (e.Delta > 0) _verticalZoom *= ZoomFactor;
            else _verticalZoom /= ZoomFactor;

            // Rescale pan offset to maintain position
            _panOffset = new Point(
                _panOffset.X,
                (int)(_panOffset.Y * _verticalZoom / oldVerticalZoom)
            );
        }
        else
        {
            float oldZoom = _zoom;
            if (e.Delta > 0)
                _zoom *= ZoomFactor;
            else
                _zoom /= ZoomFactor;

            _zoom = Math.Max(MinZoom, Math.Min(MaxZoom, _zoom));

            // Center zoom on cursor
            var mousePos = e.Location;
            float zoomChange = _zoom / oldZoom;
            _panOffset = new Point(
                (int)(mousePos.X - zoomChange * (mousePos.X - _panOffset.X)),
                (int)(mousePos.Y - zoomChange * (mousePos.Y - _panOffset.Y))
            );
        }

        this.Invalidate();
    }
}

const startTime = 0;
const endTime = 24 * 60 * 60;
const vbWidth = 300;
const vbHeight = 150;
const padding = 10;

const scaleX = (x) => {
  return x / (endTime + startTime) * (vbWidth - padding * 2);
}


function parseTime(s) {
  const [h, m] = s.split(':');
  return h * 3600 + m * 60;
}

function generate(id = "json-input") {
  let input
  try {
    input = JSON.parse(document.getElementById(id).value);
  }
  catch {
    console.error("Input is not valid JSON")
  }
  
  input.trips.forEach((trip, index) => {
    trip.sTime = parseTime(trip.startTime);
    trip.eTime = parseTime(trip.endTime);
    trip.posX = scaleX(trip.sTime);
    trip.posY = padding + (1 + index) / (2 + input.trips.length) * (vbHeight - 2 * padding) ;
    trip.width = (trip.eTime - trip.sTime) / endTime * (vbWidth - 2 * padding) 
  });

  const trips = input.trips.map((trip) => `
    <text class="trip" x="${trip.posX}" y="${trip.posY - 3}">${trip.route} (${trip.id})</text>
    <rect x="${trip.posX}" y="${trip.posY}" width="${trip.width}" height="6" rx="3" />
  `).join('')

  const depots = input.depots.map((depot, index) => {
    const posXStart = padding;
    const posXEnd = vbWidth - padding - 6;
    const posY = padding + (1 + index) / (2 + input.depots.length) * (vbHeight - 2 * padding)
    return `
      <rect x="${posXStart}" y="${posY}" height="${10}" width="6" ry="3" />
      <text class="trip" x="${posXStart}" y="${posY - 3}">${depot.id}</text>
      <rect x="${posXEnd}" y="${posY}" height="${10}" width="6" ry="3" />
      <text class="trip" x="${posXEnd}" y="${posY - 3}">${depot.id}</text>
    `
  })

  const svg = `
    <svg viewBox="0 0 ${vbWidth} ${vbHeight}" fill="white" xmlns="http://www.w3.org/2000/svg">
      ${trips}
      ${depots}
    </svg>
  `
  const container = document.getElementById('svg-container');
  container.innerHTML = svg;
}
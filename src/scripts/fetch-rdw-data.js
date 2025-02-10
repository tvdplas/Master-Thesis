const ALL_BUSES_URL = "https://opendata.rdw.nl/resource/m9d7-ebf2.json?voertuigsoort=Bus&$limit=10000000&$select=kenteken"
const TYPE_BASE_URL = "https://opendata.rdw.nl/resource/8ys7-d773.json?$select=brandstof_omschrijving&kenteken="

async function run() {
  const all_buses = await fetch(ALL_BUSES_URL)
  const all_buses_license = JSON.parse(await all_buses.text()).map(({ kenteken }) => kenteken)
  
  const types = {}
  for (const license of all_buses_license) {
    const [ { brandstof_omschrijving: bo } ]  = JSON.parse(await (await fetch(TYPE_BASE_URL + license)).text());
    if (types[bo]) types[bo]++;
    else types[bo] = 1;    
    console.log(types)
  }

  console.log(types)
}

run()
const fs = require("node:fs");
const path = require("node:path");
const fengari = require("fengari");

const root = path.resolve(__dirname, "..");
process.chdir(root);
const L = fengari.lauxlib.luaL_newstate();
fengari.lualib.luaL_openlibs(L);

function errorText() {
  return fengari.to_jsstring(fengari.lua.lua_tostring(L, -1));
}

function compileFile(file) {
  const source = fs.readFileSync(file, "utf8");
  const code = fengari.lauxlib.luaL_loadbufferx(L, fengari.to_luastring(source), null, fengari.to_luastring(file), "t");
  if (code !== fengari.lua.LUA_OK) throw new Error(`${file}: ${errorText()}`);
  fengari.lua.lua_settop(L, 0);
}

for (const file of [
  "run.lua", "fcc.lua", "display.lua", "fcc-cli.lua", "oc2-bridge.lua",
  "turtle_startup.lua", "boot.lua", "modules/oc2_wire.lua",
  "modules/oc2_protocol.lua", "modules/oc2_bridge.lua", "modules/flight_core.lua",
  "tests/run.lua", "tests/oc2_wire_spec.lua", "tests/oc2_protocol_spec.lua", "tests/oc2_bridge_spec.lua"
]) {
  compileFile(file);
  console.log(`LUA PARSE OK ${file}`);
}

const testFile = path.join(root, "tests", "run.lua");
let status = fengari.lauxlib.luaL_loadfile(L, fengari.to_luastring(testFile));
if (status === fengari.lua.LUA_OK) status = fengari.lua.lua_pcall(L, 0, fengari.lua.LUA_MULTRET, 0);
if (status !== fengari.lua.LUA_OK) {
  console.error(errorText());
  process.exit(1);
}

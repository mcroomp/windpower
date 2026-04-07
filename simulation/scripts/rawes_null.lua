-- rawes_null.lua — Absolute minimum: no API calls at all.
-- If SITL crashes even with this, the issue is in Lua runtime initialization.

local function update()
    return update, 1000
end

return update, 1000

+++
title = "About"
description = "Where the parameter data comes from."
+++

Parameter data is generated from each ArduPilot firmware branch's `apm.pdef.xml`
and collected in the [ParameterRepository](https://github.com/ArduPilot/ParameterRepository),
which a bot refreshes every five days. This site searches them in the browser:
every word typed must start a word in the parameter name or description, with a
fuzzy fallback when nothing matches.

"latest" is the highest version folder available for a vehicle. Values, bitmasks,
ranges and units come straight from the metadata; defaults are board-specific and
are not part of it.

# Handles every parameter on the component: the work goes to the extensions, the
# three About buttons open a link in the system browser. Nothing here loads a
# web page inside TouchDesigner.

import webbrowser

LINKS = {
	'Readme': 'https://github.com/DarienBrito/TDXDepthCamMerger',
	'Support': 'https://www.patreon.com/c/darienbrito',
	'Website': 'https://www.darienbrito.com',
}


def onValueChange(par, prev):
	comp = parent()
	if par.name in ('Specifypair1', 'Specifypair2', 'Devices'):
		comp.SetIds()
	elif par.name == 'Devicetype':
		# Rebuild the template straight away rather than leaving the old camera
		# selects in place until the next Gather devices pulse. RebuildDevices,
		# not BuildDeviceSources: rebuilding destroys the template's operators,
		# and every clone has to be copied again afterwards.
		comp.RebuildDevices()
	return


def onPulse(par):
	comp = parent()
	name = par.name

	if name == 'Calibrate':
		comp.Calibrate(pair=comp.GetPair(), mode=comp.par.Mode.eval())
	elif name == 'Refine':
		comp.Refine(pair=comp.GetPair())
	elif name == 'Gatherdevices':
		comp.GatherDevices()
	elif name == 'Rebuildchain':
		comp.RebuildChain()
	elif name == 'Resetcalibration':
		comp.ResetCalibration()
	elif name == 'Checkworker':
		comp.CheckWorker()
	elif name in LINKS:
		webbrowser.open(LINKS[name])
	return

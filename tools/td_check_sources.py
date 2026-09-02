"""
Compare the DATs inside the built component with the repo files they came from.

td_build.py only ever copies file -> DAT, so a DAT edited inside TouchDesigner
is quietly overwritten by the next build while the repo keeps the old text. Run
this before saving, or before trusting a build.

Run from TD:  exec(open(r'<this file>', encoding='utf-8').read())
"""

import difflib
import os

REPO = globals().get('REPO') or project.folder.replace(os.sep, '/')
SRC = REPO + '/src'
COMP = op('/ProjectName/TDXDepthCamMerger')

PAIRS = (
	('extTDXDepthCamMerger', 'extTDXDepthCamMerger.py'),
	('extUtilities', 'extUtilities.py'),
	('workerSource', 'worker.py'),
	('Device1/extDevice', 'extDevice.py'),
)

if COMP is None:
	raise RuntimeError('no component at /ProjectName/TDXDepthCamMerger')

drifted = []
for datPath, fileName in PAIRS:
	dat = COMP.op(datPath)
	path = SRC + '/' + fileName
	if dat is None:
		drifted.append(datPath)
		print('  [MISSING DAT] {}'.format(datPath))
		continue
	if not os.path.isfile(path):
		drifted.append(datPath)
		print('  [MISSING FILE] {}'.format(path))
		continue
	# Comparing line by line hides a trailing newline and any line ending the
	# DAT picked up, so only real changes show up.
	onDisk = open(path, encoding='utf-8').read().splitlines()
	inTd = dat.text.splitlines()
	if onDisk == inTd:
		print('  [OK] {} == src/{}'.format(datPath, fileName))
		continue
	drifted.append(datPath)
	print('  [DRIFT] {} != src/{}'.format(datPath, fileName))
	for line in difflib.unified_diff(onDisk, inTd, 'src/' + fileName,
	                                 'DAT ' + datPath, lineterm=''):
		print('    ' + line)

print('')
if drifted:
	print('DRIFT in {}: {}'.format(len(drifted), ', '.join(drifted)))
	print('The DAT is what runs. Copy it back into src/ if the DAT is right, or')
	print('re-run tools/td_build.py if the file is right.')
else:
	print('All {} sources match {}'.format(len(PAIRS), SRC))

"""
Compare the DATs inside the built component against the repo sources they were
installed from.

td_build.py only ever copies file -> DAT. A DAT edited inside TouchDesigner is
therefore silently reverted by the next build, and the repo keeps shipping the
old text. Run this before saving, or before trusting a build.

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
	# splitlines absorbs a trailing newline and any CRLF the DAT picked up, so
	# only real content differences show up.
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

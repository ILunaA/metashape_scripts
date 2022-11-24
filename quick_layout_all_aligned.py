import os
import Metashape
from PySide2 import QtWidgets

global app
app = QtWidgets.QApplication.instance()
doc = Metashape.app.document

chunk = doc.chunk
crs = chunk.crs
T = chunk.transform.matrix
print('Processing started...')
nprocessed = 0

if chunk.model:
	surface = chunk.model
elif chunk.dense_cloud:
	surface = chunk.dense_cloud
else:
	surface = chunk.point_cloud

for photo in chunk.cameras:
	if not photo.type == Metashape.Camera.Type.Regular: #skip camera track, if any
		continue
	if not photo.transform: #skip not aligned cameras
		continue

	A = D = B = E = C = F = 0.0
	x0 = Metashape.Vector((0.0,0.0,0.0))
	x1 = Metashape.Vector((0.0,0.0,0.0))
	x2 = Metashape.Vector((0.0,0.0,0.0))

	if (photo.photo.path.find("/") == -1):
		name = photo.photo.path.rsplit("\\", 1)[1]
	else:
		name = photo.photo.path.rsplit("/", 1)[1]
	path = photo.photo.path.rsplit(".", 1)[0]
	photo_ext = photo.photo.path.rsplit(".",1)[1]

	file_ext = ".jgw"
	if (photo_ext.upper() in ["TIF", "TIFF"]):
		file_ext = ".tfw"
	elif (photo_ext.upper() in ["JPG", "JPEG"]):
		file_ext = ".jgw"
	file = open(path + file_ext, "wt")

	# vectors corresponding to photo corners

	sensor = photo.sensor
	width = photo.sensor.width
	height = photo.sensor.height
	calib = photo.sensor.calibration
	corners = list()
	for i in [[0, 0], [sensor.width - 1, 0], [sensor.width - 1, sensor.height - 1], [0, sensor.height - 1]]:
		corners.append(surface.pickPoint(photo.center, photo.transform.mulp(calib.unproject(Metashape.Vector(i)))))
		if not corners[-1]:
			corners[-1] = chunk.point_cloud.pickPoint(photo.center, photo.transform.mulp(calib.unproject(Metashape.Vector(i))))
		if not corners[-1]:
			break
		corners[-1] = crs.project(T.mulp(corners[-1]))

	if not all(corners):
		print("Skipping camera " + photo.label + "...")
		continue

	x0 = corners[0]
	x1 = corners[1]
	x2 = corners[3]

	# solution
	C = x0[0]
	F = x0[1]
	A = (x1[0] - x0[0]) / width
	D = (x1[1] - x0[1]) / width
	B = (x2[0] - x0[0]) / height
	E = (x2[1] - x0[1]) / height

	n = "\n"
	output_str = ("{0:.10f}".format(A) + n + "{0:.10f}".format(D) + n + "{0:.10f}".format(B) + n + "{0:.10f}".format(E) + n + "{0:.10f}".format(C) + n + "{0:.10f}".format(F) + n)
	file.write(output_str)
	file.close()
	print(photo.label + " written...")
	app.processEvents()
	nprocessed += 1

print('Processing finished, generated ' + str(nprocessed) + ' world file(s)')

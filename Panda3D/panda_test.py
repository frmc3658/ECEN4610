from panda3d.core import Point3
from direct.showbase.ShowBase import ShowBase
from panda3d.vision import ARToolKit

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        # Initialize ARToolKit with the correct configuration files
        self.ar_toolkit = ARToolKit(self)
        self.ar_toolkit.loadParam("camera_para.dat")
        self.ar_toolkit.loadMarkers("camera_para.dat")

        # Set up the camera
        self.ar_toolkit.attachCamera(self.cameraNode)

        # Load a 3D model
        self.model = self.loader.loadModel("test.egg")
        self.model.reparentTo(self.render)
        self.model.setPos(0, 0, 0)  # Initial position of the model

        # Create a marker
        self.marker = self.ar_toolkit.createMarker(0, 16, 40)
        self.marker.reparentTo(self.render)

        # Define a function to update the model's position based on the marker
        def update_model(task):
            self.ar_toolkit.update(self.marker)

            if self.ar_toolkit.getMarkerNum(self.marker) > 0:

                # Get the marker's transformation matrix
                matrix = self.ar_toolkit.getTransform(self.marker)
                
                # Extract position and rotation from the matrix
                position = matrix.getTranslate()
                rotation = matrix.getRotate()

                # Update the model's position and rotation
                self.model.setPos(Point3(position[0], position[1], position[2]))
                self.model.setHpr(rotation[0], rotation[1], rotation[2])

            return task.cont

        # Add the update function to the task manager
        self.taskMgr.add(update_model, "update_model")

app = MyApp()
app.run()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import face_recognition
import PIL.Image as PilImage
import numpy as np
from docarray import DocumentArray, Document
from annlite import AnnLite
import time
# chroma_client = chromadb.Client()

FACE_VECTOR_DIMS = 128

FIRST_NAMES = [
  "John",
  "Jane",
  "Alice",
  "Bob",
  "Charlie",
  "David",
  "Eve",
  "Frank",
  "Grace",
  "Heidi",
  "Ivan",
  "Judy",
]

SILLY_LAST_NAMES = [
  "Obeliskivitsky",
  "Foosh",
  "Crunk",
  "Mud",
  "Pickle",
  "Picklestein",
  "Picklesteinberg",
  "Picklesteinberger",
  "Picklesteinbergerstein",
  "Picklesteinbergersteinberg",
  "Picklesteinbergersteinberger",
]

def generate_random_name():
  return f"{np.random.choice(FIRST_NAMES)} {np.random.choice(SILLY_LAST_NAMES)}"

class ImageIdentifier(Node):
  """
  Create an ImageIdentifier class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_identifier', namespace="railbot")

    # used to fetch video frames
    self.subscription = self.create_subscription(
      Image,
      'video_frames',
      self.listener_callback,
      10)

    # used to classify faces
    self.face_classifier = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )

    # used to store recognized faces
    self.ann = AnnLite(128, metric='cosine', columns=[('timestamp', float)], data_path="/tmp/annlite_data")

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Identifier receiving video frame')

    identify_opencv = False

    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)

    if identify_opencv:
      # this seems to be a faster alternative if you don't want vectors... but we do, I think.
      gray_image = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
      face = self.face_classifier.detectMultiScale(
          gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
      )
      for (x, y, w, h) in face:
        cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 255, 0), 4)

    face_frame_array = np.asarray(PilImage.fromarray( cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)))
    face_locations = face_recognition.face_locations(face_frame_array)
    face_encodings = face_recognition.face_encodings(face_frame_array, face_locations)

    print("face locations", face_locations)

    # render onto the image
    for (top, right, bottom, left) in face_locations:
      cv2.rectangle(current_frame, (left, top), (right, bottom), (0, 0, 255), 2)

    # uncomment to display the image
    cv2.imshow("camera", current_frame)
    print(face_encodings)

    # look up faces
    if len(face_encodings) > 0:
      # query for all the faces at once, picking the top match for each face
      query = DocumentArray.empty(len(face_encodings))
      query.embeddings = face_encodings
      result = self.ann.search(query, limit=1)
      print("search result", result)
      query_results = query['@m', ('id', 'scores__cosine', 'tags__name', 'tags__location', 'tags__timestamp')]
      # [['f9fbebf22459a35295d8348c0b1bd5a1', '9d10185387ebb5df9bad6877d6760232'], [{'value': 0.007400036}, {'value': 0.010317743}], ['Grace Mud', 'Eve Picklesteinbergersteinberg']]
      print("query results", query_results)

      top_match_for_encodings = [(q.matches[0] if len(q.matches) > 0 else None) for q in query]
      new_docs_to_index = []
      new_docs_embeddings = []

      for idx, m in enumerate(top_match_for_encodings):
        if m is not None and m.scores["cosine"].value < 0.02:
          # TODO: take some action based on recognizing someone! record it in a log at least
          continue

        # print(f'match {m.id} {m.scores["cosine"]} {m.tags["name"]} {m.location}')
        # if the score is < 0.02, consider it a match
        # if :



        # if the score is > 0.02, but the best match occurred < 5 seconds ago, create a new face but copy the name from the old one
        if m is not None and m.tags['timestamp'] and time.time() - m.tags['timestamp'] < 5:
          # copy the name of the last person we saw
          # TODO: link the new face to the old one in the database
          name = m.tags["name"]
        else:
          name = generate_random_name()

        new_doc = Document(
          tags={"name": name, "location": face_locations[idx], "timestamp": time.time()},
        )

        new_docs_to_index.append(new_doc)
        new_docs_embeddings.append(face_encodings[idx])


      if len(new_docs_to_index) > 0:
        docs = DocumentArray(
          new_docs_to_index
        )
        docs.embeddings = new_docs_embeddings
        self.ann.index(docs)


      # to find new faces, we

      # for q in query:
      #   print(f'Query {q.id}', q)

      #   has_match = False
      #   for k, m in enumerate(q.matches):
      #       # if the previous
      #       if m.scores["cosine"].value < 0.02:
      #         has_match = True
      #       print(f'{k}: {m.id} {m.scores["cosine"]} {m.tags["name"]} {m.location}')

      # index the current faces if they are not recognized


      # add the face encoding to the ann
      # self.ann.add(face_encodings[0])

    # uncomment to kill the opencv window
    cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageIdentifier()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
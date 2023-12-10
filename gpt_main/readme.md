The `gpt_main` package provides a set of ROS2 nodes for integrating the OpenAI GPT (Generative Pre-trained Transformer) models, such as GPT-3.5-turbo or GPT-4, into a ROS2-based system. This package is specially designed to handle natural language processing and chatbot applications using the OpenAI API. The package consists of three primary scripts:

- `gpt_ros2_client_demo.py`
- `gpt_ros2_server_demo.py`
- `gpt_service.py`

## gpt_ros2_client_demo.py

This script defines a ROS2 client node that communicates with a GPTText service. The client node reads user input as a text prompt, sends it to the GPTText service, and retrieves a response text from it. The retrieved response is then displayed to the user as the "Mini Pupper" reply. The client node continuously reads user input and sends requests to the GPTText service until terminated. This example script demonstrates the usage of a custom ROS2 service to communicate with GPT-3.5-turbo or GPT-4 and integrate it into a ROS2-based system.

## gpt_ros2_server_demo.py

This script defines a ROS2 server node which provides a GPTText service based on the GPT-3.5-turbo or GPT-4 model from OpenAI. The GPTText service takes a user input as text prompt, generates chat completion using OpenAI API, and returns a chat response. The server node relies on a GPTConfig module to manage LLM model configurations, including handling chat history and API settings. A client node (such as the one in the previous example) can communicate with this server node to receive AI-generated responses. This example script is useful for integrating a LLM, using the OpenAI API, within a ROS2-based system for natural language processing and chatbot applications.

## gpt_service.py

This script creates a ROS2 node named "gpt_service" that listens to a topic for text input, processes it using the OpenAI GPT API, and publishes the generated text response. When receiving a text message from the user, the node preprocesses it, generates a response, and appends the user's input and GPT assistant's response to the chat history. The GPTService class includes a callback to handle the text input and methods to handle general input processing, generating chat completions, and creating response text. The main function initializes and runs the GPTService node.


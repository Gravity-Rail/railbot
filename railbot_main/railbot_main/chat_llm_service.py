#!/usr/bin/env python3

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# GPT related
from openai import OpenAI

# GPT status related
from railbot_status.railbot_param_server import RailbotStatus, RailbotStatusOperation

# GPT config related
from railbot_status.railbot_config import RailbotConfig

from typing import Optional

from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.runnables.history import RunnableWithMessageHistory
from langchain_openai import ChatOpenAI

config = RailbotConfig()
chat_history = [] # TODO: sqlite

# openai.organization = config.organization

client = OpenAI(api_key=config.api_key)

# TODO:
# * generalize to any Chat model supported by Langchain

class ChatLLMService(Node):
    def __init__(self):
        super().__init__("chat_llm_service", namespace="railbot")
        self.subscription = self.create_subscription(
            String, "chat_llm_text_input", self.llm_callback, 10
        )
        self.publisher = self.create_publisher(String, "chat_llm_text_output", 10)

        # initialize history with the system prompt
        self.append_message_to_history("system", config.system_prompt)

        self.status_operation = RailbotStatusOperation()

    def llm_callback(self, msg):
        self.get_logger().info("ChatLLM node received: %s" % msg.data)
        self.status_operation.set_railbot_status_value(RailbotStatus.CHAT_LLM_PROCESSING.name)

        # prompt = ChatPromptTemplate.from_messages(
        #     [
        #         ("system", config.system_prompt),
        #         MessagesPlaceholder(variable_name="history"),
        #         ("human", "{question}"),
        #     ]
        # )

        # chain = prompt | ChatOpenAI()


        input = self.user_input_processor(msg.data)
        llm_response = self.generate_chat_completion(input)
        output = self.get_chat_response_text(llm_response)

        self.append_message_to_history("user", config.user_prompt)
        self.append_message_to_history("assistant", config.assistant_response)

        # Publish the response
        response_msg = String(data=output)
        self.publisher.publish(response_msg)
        self.get_logger().info(
            "Chat LLM service node has published: %s" % response_msg
        )

    def user_input_processor(self, user_prompt):
        """
        This function takes the user prompt
        and preprocesses it to be used as input
        """
        input = []
        for message in chat_history:
            input.append(
                {"role": message["role"], "content": message["content"]}
            )
        config.user_prompt = user_prompt
        input.append({"role": "user", "content": config.user_prompt})
        return input

    def generate_chat_completion(self, input):
        """
        This function takes the input and generates the chat completion
        and returns the response
        """
        try:
            response = client.chat.completions.create(model=config.model,
            messages=input,
            temperature=config.temperature,
            max_tokens=config.max_tokens,
            top_p=config.top_p,
            frequency_penalty=config.frequency_penalty,
            presence_penalty=config.presence_penalty,
            stop=config.stop)
        except Exception as e:
            # Handle the error as per your requirement
            self.get_logger().error(f"Error: {e}")
            response = None
        return response

    def get_chat_response_text(self, response):
        """
        This function takes the response
        and returns the chat response text individually
        """
        response_text = response.choices[0].message.content.strip()
        config.assisstant_response = response_text
        return response_text

    def append_message_to_history(self, user_or_ai, content):
        chat_history.append({"role": user_or_ai, "content": content})


def main(args=None):
    rclpy.init(args=args)
    llm_service = ChatLLMService()
    rclpy.spin(llm_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

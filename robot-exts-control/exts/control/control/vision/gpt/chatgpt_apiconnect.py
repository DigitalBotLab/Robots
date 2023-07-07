# SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import omni.usd
import carb
import os
import aiohttp
import asyncio
from pxr import Sdf
from .prompts import system_input, user_input, assistant_input
from .deep_search import query_items
from .item_generator import place_greyboxes, place_deepsearch_results


async def chatGPT_call(prompt: str):
    # Load your API key from an environment variable or secret management service
    settings = carb.settings.get_settings()
    
    apikey = settings.get_as_string("/persistent/exts/omni.example.airoomgenerator/APIKey")
    my_prompt = prompt.replace("\n", " ")
    
    # Send a request API
    try:
        parameters = {
            "model": "gpt-3.5-turbo",
            "messages": [
                    {"role": "system", "content": system_input},
                    {"role": "user", "content": user_input},
                    {"role": "assistant", "content": assistant_input},
                    {"role": "user", "content": my_prompt}
                ]
        }
        chatgpt_url = "https://api.openai.com/v1/chat/completions"
        headers = {"Authorization": "Bearer %s" % apikey}
        # Create a completion using the chatGPT model
        async with aiohttp.ClientSession() as session:
            async with session.post(chatgpt_url, headers=headers, json=parameters) as r:
                response = await r.json()
        text = response["choices"][0]["message"]['content']
    except Exception as e:
        carb.log_error("An error as occurred")
        return None, str(e)

    # Parse data that was given from API
    try: 
        #convert string to  object
        data = json.loads(text)
    except ValueError as e:
        carb.log_error(f"Exception occurred: {e}")
        return None, text
    else: 
        # Get area_objects_list
        object_list = data['area_objects_list']
        
        return object_list, text

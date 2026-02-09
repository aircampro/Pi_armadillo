#!/usr/bin/env python
#
# Answer a SIP voice call and translate the incoming audio to text using whisper.
# if we find the phrase "add" then add that user to Zoho CRM else if user says phrase "delete" remove the said user
#
from pyVoIP.VoIP import VoIPPhone, CallState
import uuid
from openai import OpenAI
import os
import pywav

# Zoho CRM https://www.zoho.com/sites/zweb/images/crm/python_v3.xx.pdf
# ref:- add a user https://www.zoho.com/crm/developer/docs/python-sdk/v1/user-samples.html?src=add_user
#
from zcrmsdk.src.com.zoho.crm.api.users import *
from zcrmsdk.src.com.zoho.crm.api.users import User as ZCRMUser
from zcrmsdk.src.com.zoho.crm.api.roles import Role
from zcrmsdk.src.com.zoho.crm.api.profiles import Profile
from zcrmsdk.src.com.zoho.crm.api import ParameterMap, HeaderMap
from datetime import datetime

class ZohoUser(object):
    @staticmethod
    def create_user(firstn="john", lastn="kelvin"):
        """
        This method is used to add a user to your organization and print the response.
        """
        # Get instance of UsersOperations Class
        users_operations = UsersOperations()

        # Get instance of RequestWrapper Class that will contain the request body
        request = RequestWrapper()
        # List to hold User instances
        user_list = []
        # Get instance of User Class
        user = ZCRMUser()
        # Get instance of Role Class
        role = Role()

        # Set ID to Role instance
        role.set_id(3409643000000026008)                                                 # all have same role
        # Set role instance to role in User
        user.set_role(role)
        user.set_country_locale('en_US')
        user.set_first_name(firstn)
        user.set_last_name(lastn)
        user.set_email(f"{firstn}.{lastn}@zoho.com")

        # Get instance of Profile Class
        profile = Profile()
        uniq = datetime.now().strftime('%Y%m%d%H%M%S')                                   # create a uniquw id for the user profile
        #profile.set_id(3409643000000395047)
        profile.set_id(uniq)
        # Set profile instance to profile in User instance
        user.set_profile(profile)
        # Add the User instance to list
        user_list.append(user)
        # Set the list to users in BodyWrapper instance
        request.set_users(user_list)

        # Call create_user method that takes RequestWrapper class instance as parameter
        response = users_operations.create_user(request)

        if response is not None:
            # Get the status code from response
            print('Status Code: ' + str(response.get_status_code()))

            # Get object from response
            response_object = response.get_object()

            if response_object is not None:

                # Check if expected ActionWrapper instance is received.
                if isinstance(response_object, ActionWrapper):

                    # Get the list of obtained ActionResponse instances
                    action_response_list = response_object.get_users()

                    for action_response in action_response_list:

                        # Check if the request is successful
                        if isinstance(action_response, SuccessResponse):
                            # Get the Status
                            print("Status: " + action_response.get_status().get_value())
                            # Get the Code
                            print("Code: " + action_response.get_code().get_value())
                            print("Details")
                            # Get the details dict
                            details = action_response.get_details()
                            for key, value in details.items():
                                print(key + ' : ' + str(value))

                            # Get the Message
                            print("Message: " + action_response.get_message().get_value())

                        # Check if the request returned an exception
                        elif isinstance(action_response, APIException):
                            # Get the Status
                            print("Status: " + action_response.get_status().get_value())
                            # Get the Code
                            print("Code: " + action_response.get_code().get_value())
                            print("Details")
                            # Get the details dict
                            details = action_response.get_details()

                            for key, value in details.items():
                                print(key + ' : ' + str(value))

                            # Get the Message
                            print("Message: " + action_response.get_message().get_value())

                # Check if the request returned an exception
                elif isinstance(response_object, APIException):
                    # Get the Status
                    print("Status: " + response_object.get_status().get_value())
                    # Get the Code
                    print("Code: " + response_object.get_code().get_value())
                    print("Details")
                    # Get the details dict
                    details = response_object.get_details()

                    for key, value in details.items():
                        print(key + ' : ' + str(value))

                    # Get the Message
                    print("Message: " + response_object.get_message().get_value())

    @staticmethod
    def delete_user(user_id):
        """
        This method is used to delete a user from your organization and print the response.
        :param user_id: The ID of the User to be deleted
        """

        """
        example
        user_id = 3409643000000302031
        """
        # Get instance of UsersOperations Class
        users_operations = UsersOperations()

        # Call delete_user method that takes user_id as parameter
        response = users_operations.delete_user(user_id)

        if response is not None:
            # Get the status code from response
            print('Status Code: ' + str(response.get_status_code()))
            # Get object from response
            response_object = response.get_object()

            if response_object is not None:

                # Check if expected ActionWrapper instance is received.
                if isinstance(response_object, ActionWrapper):

                    # Get the list of obtained ActionResponse instances
                    action_response_list = response_object.get_users()

                    for action_response in action_response_list:

                        # Check if the request is successful
                        if isinstance(action_response, SuccessResponse):
                            # Get the Status
                            print("Status: " + action_response.get_status().get_value())
                            # Get the Code
                            print("Code: " + action_response.get_code().get_value())
                            print("Details")
                            # Get the details dict
                            details = action_response.get_details()
                            for key, value in details.items():
                                print(key + ' : ' + str(value))

                            # Get the Message
                            print("Message: " + action_response.get_message().get_value())

                        # Check if the request returned an exception
                        elif isinstance(action_response, APIException):
                            # Get the Status
                            print("Status: " + action_response.get_status().get_value())
                            # Get the Code
                            print("Code: " + action_response.get_code().get_value())
                            print("Details")
                            # Get the details dict
                            details = action_response.get_details()

                            for key, value in details.items():
                                print(key + ' : ' + str(value))

                            # Get the Message
                            print("Message: " + action_response.get_message().get_value())

                # Check if the request returned an exception
                elif isinstance(response_object, APIException):
                    # Get the Status
                    print("Status: " + response_object.get_status().get_value())
                    # Get the Code
                    print("Code: " + response_object.get_code().get_value())
                    print("Details")
                    # Get the details dict
                    details = response_object.get_details()

                    for key, value in details.items():
                        print(key + ' : ' + str(value))

                    # Get the Message
                    print("Message: " + response_object.get_message().get_value())

    @staticmethod
    def get_user_id(fir, last):
        """
        This method is used to retrieve the user id for the requested user
        """
        # Get instance of UsersOperations Class
        users_operations = UsersOperations()
        # Get instance of ParameterMap Class
        param_instance = ParameterMap()
        # Possible parameters for Get Users operation
        param_instance.add(GetUsersParam.page, 1)
        param_instance.add(GetUsersParam.per_page, 200)
        param_instance.add(GetUsersParam.type, 'ActiveConfirmedUsers')
        # Get instance of ParameterMap Class
        header_instance = HeaderMap()
        # Possible headers for Get Users operation
        header_instance.add(GetUsersHeader.if_modified_since, datetime.fromisoformat('2019-07-07T10:00:00+05:30'))
        # Call get_users method that takes ParameterMap instance and HeaderMap instance as parameters
        response = users_operations.get_users(param_instance, header_instance)

        if response is not None:

            # Get the status code from response
            print('Status Code: ' + str(response.get_status_code()))

            if response.get_status_code() in [204, 304]:
                print('No Content' if response.get_status_code() == 204 else 'Not Modified')
                return

            # Get object from response
            response_object = response.get_object()

            if response_object is not None:

                # Check if expected ResponseWrapper instance is received.
                if isinstance(response_object, ResponseWrapper):

                    # Get the list of obtained User instances
                    user_list = response_object.get_users()

                    for user in user_list:
                        id=str(user.get_id())
                        fn=str(user.get_first_name())
                        ln=str(user.get_last_name())
                        if fn == fir and ln == last:                                # requested user has been found then return the id
                            return id
                    return -1
                    
                # Check if the request returned an exception
                elif isinstance(response_object, APIException):
                    # Get the Status
                    print("Status: " + response_object.get_status().get_value())
                    # Get the Code
                    print("Code: " + response_object.get_code().get_value())
                    print("Details")
                    # Get the details dict
                    details = response_object.get_details()

                    for key, value in details.items():
                        print(key + ' : ' + str(value))

                    # Get the Message
                    print("Message: " + response_object.get_message().get_value())
                    return -1
            else:
                return -1
        else:
            return -1
            
REC_LEN=2000                                                                                                    # recirding length 2 second

def convert_to_wav(audio, tmpFileName):
    data_bytes = b"".join(audio)
    wave_write = pywav.WavWrite(tmpFileName, 1, 8000, 8, 7)
    wave_write.write(data_bytes)
    wave_write.close()
    return open(tmpFileName, "rb")

# send the chunk audio to whisper for transcription
#
def transcribe_to_text(audio_file):
    tmpFileName = f"/tmp/audio/_audio_buffer_{uuid.uuid4()}.wav"
    client = OpenAI()

    transcription = client.audio.transcriptions.create(
        model="whisper-1",
        file=convert_to_wav(audio_file, tmpFileName)
    )
    try:
        return transcription.text
    except Exception as ex:
       print(ex)
    return ""

# buffer chunks of audio as 100ms and send to whisper-1
# In our "answer" method we receive the audio as a continuous stream of bytes, 
# therefore each chunk of audio is 20ms. We cannot send a 20ms chunk of audio to Whisper because the minimum length is 100ms.
# Thus, we need to append the audio to a buffer and we'll only send the audio to Whisper once we reach 1000ms (or 1 second).
#
def answer(call):
    try:
        call.answer()
        buffer = []
        buff_length = 0
        state = 0
        action=0
        z = ZohoUser()                                                     # interface to the zoho CRM
        while call.state == CallState.ANSWERED:
            audio = call.read_audio()
            buff_length += len(audio) / 8

            if buff_length <= REC_LEN:
                buffer.append(audio)
            else:                                                          # process speach (add | delete) -> (first name) -> (end) -> (second name) -> (complete) or (reset)
                t=transcribe_to_text(buffer)
                if not t.find("add") == -1 and state == 0:
                    state = 1
                    action = 1
                    first=""
                elif not t.find("end") == -1 and state == 1:
                    state = 2
                    second = ""
                elif state == 1:
                    first+=t   
                elif not t.find("complete") == -1 and state == 2:
                    state = 0
                elif state == 2:
                    second+=t 

                if not t.find("delete") == -1 and state == 0:
                    state = 10
                    action = 2
                    first=""
                elif not t.find("end") == -1 and state == 10:
                    state = 20
                    second = ""
                elif state == 10:
                    first+=t   
                elif not t.find("complete") == -1 and state == 20:
                    state = 0
                elif state == 20:
                    second+=t 
                elif not t.find("reset") == -1:
                    state = 0                    
                buffer = []
                buff_length = 0
        if action == 1 and not first == None and not second == None:
            z.create_user(first, second)
        elif action == 2 and not first == None and not second == None:
            id = z.get_user_id(first, second)
            if id != -1:
                 z.delete_user(id)
            else:
                 print("unable to find that user requested")
                 
    except Exception as e:
        print(e)
    finally:
       call.hangup()

SIP_Server_IP=sip.linphone.org
SIP_Server_PORT=5060
SIP_Server_Username="uruser"
SIP_Pass='sippassword'
vp = VoIPPhone(SIP_Server_IP, SIP_Server_PORT, SIP_Server_Username, SIP_Pass, callCallback=answer)
vp.start()
print(vp._status)
input("Press any key to hang-up phone line")
vp.stop()
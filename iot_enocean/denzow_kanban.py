# connect to this custom github kanban server 
# you need to install this github project https://github.com/denzow/channel-kanban/tree/master/application/modules/kanban
# also ref this blog https://qiita.com/denzow/items/aa8f01222fa356b814a3
#
# this lib forms the client part and we will use it in iot_telem
#
import json
from channels.generic.websocket import AsyncWebsocketConsumer
from modules.kanban import service as kanban_sv

class KanbanConsumer(AsyncWebsocketConsumer):
    """
    WebSocket connection to danzow kanban server
    """
:
    async def connect(self):
        self.kanban_id = self.scope['url_route']['kwargs']['kanban_id']
        self.kanban_name = 'kanban_{}'.format(self.kanban_id)

        # Join room group
        await self.channel_layer.group_add(
            self.kanban_name,
            self.channel_name
        )
        await self.accept()
        await self.send(text_data=json.dumps({
            'kanban': kanban_sv.get_whole_json(self.kanban_id),
            'type': 'set_data',
        }))
:
    async def receive(self, text_data=None, bytes_data=None):
        text_data_json = json.loads(text_data)
        message_type = text_data_json['type']
        payload = text_data_json['payload']
        await self.type_map[message_type](payload)
:
    async def updated(self, event):
        payload = event['payload']
        await self.send(text_data=json.dumps({
            'type': 'set_data',
            'kanban': kanban_sv.get_whole_json(self.kanban_id),
        }))
:
    async def _update(self, payload):
        print('_update', payload)
        kanban_sv.update_kanban(
            pipeline_id=payload['pipeLineId'],
            card_id_list=[x['id'] for x in payload['newCardList']]
        )
        # Send message to room group
        await self.channel_layer.group_send(
            self.kanban_name,
            {
                'type': 'updated',
                'payload': {}
            }
        )

    async def _add_card(self, payload):
        kanban_sv.add_card(
            pipeline_id=payload['pipeLineId'],
            title=payload['title'],
            order=payload['order'],
        )
        # Send message to room group
        await self.channel_layer.group_send(
            self.kanban_name,
            {
                'type': 'updated',
                'payload': {}
            }
        )

    async def _add_pipeline(self, payload):
        kanban_sv.add_pipeline(
            kanban_id=payload['kanbanId'],
            title=payload['title'],
            order=payload['order'],
        )
        # Send message to room group
        await self.channel_layer.group_send(
            self.kanban_name,
            {
                'type': 'updated',
                'payload': {}
            }
        )

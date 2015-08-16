var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});
ros.on('connection', function() {
  console.log('Connected to websocket server.');
});
ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});
ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

var view_publisher = new ROSLIB.Topic({
  ros: ros,
  name: 'pr2_pick_main/web_interface/interface_submission',
  messageType: 'pr2_pick_main/web_interface/WebInterfaceSubmission'
});

if (Meteor.isClient) {
  Template.body.helpers({
    interface_type: function() {
      return Session.get('interface_type');
    },
    interface_params: function() {
      return Session.get('interface_params');
    }
  });

  Template.body.onCreated(function() {
    var that = this;
    Session.setDefault('interface_type', 'default');
    var default_params = {}
    default_params['interface_name'] = 'Robot'
    Session.setDefault('interface_params', default_params);

    this.view_listener = new ROSLIB.Topic({
      ros: ros,
      name: 'pr2_pick_main/web_interface/interface_params',
      messageType: 'pr2_pick_main/web_interface/WebInterfaceParams'
    });

    this.view_listener.subscribe(function(message) {
      Session.set('interface_type', message.interface_type);
      var params = {}
      params['interface_name'] = message.interface_name
      for (var i=0; i<message.keys.length; i+=1) {
        var key = message.keys[i];
        var value = message.values[i];
        if (message.interface_type === 'ask_choice') {
          if (key === 'choices') {
            value = JSON.parse(value);
          }
          if (key === 'prompt_id') {
            Session.set('prompt_id', value);
          }
        }
        params[key] = value;
      }
      Session.set('interface_params', params);
    });

  });

  Template.ask_choice.events({
    'click .select-choice': function(event) {
      var submission = new ROSLIB.Message({
        interface_type: 'ask_choice',
        keys: ['choice', 'prompt_id'],
        values: [event.target.value, Session.get('prompt_id')]
      });
      view_publisher.publish(submission);
    
      return false;
    }
  });
}

if (Meteor.isServer) {
  Meteor.startup(function () {
    // code to run on server at startup
  });
}

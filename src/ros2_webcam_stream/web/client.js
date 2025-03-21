// Global variables
let pc = null;

// Get HTML elements
const startButton = document.getElementById('start');
const videoElement = document.getElementById('video');
const statusElement = document.getElementById('status');

// Set up event listeners
startButton.addEventListener('click', start);

async function start() {
    if (pc) {
        return;
    }

    // Create peer connection
    pc = new RTCPeerConnection();

    // Handle ICE connection state
    pc.oniceconnectionstatechange = function() {
        console.log("ICE connection state:", pc.iceConnectionState);
    };

    // Handle incoming tracks
    pc.ontrack = function(evt) {
        if (evt.track.kind === 'video') {
            videoElement.srcObject = evt.streams[0];
            videoElement.play();
            updateStatus('Connected, receiving video');
        }
    };

    try {
        updateStatus("Creating offer...");
        // Add explicit video transceiver (recvonly) to properly receive video track in modern browsers.
        pc.addTransceiver("video", { direction: "recvonly" });
        const offer = await pc.createOffer();
       
        // Set local description
        await pc.setLocalDescription(offer);

        // Send offer to server
        const response = await fetch('/offer', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                sdp: pc.localDescription.sdp,
                type: pc.localDescription.type
            })
        });
        
        // Get answer
        const answer = await response.json();
        updateStatus('Received answer, connecting...');
        
        // Apply remote description
        await pc.setRemoteDescription(answer);
        
        // Disable start button
        startButton.disabled = true;
    } catch (e) {
        console.error('Error establishing connection:', e);
        updateStatus('Connection failed: ' + e);
        
        // Clean up failed connection
        if (pc) {
            pc.close();
            pc = null;
        }
    }
}

function updateStatus(message) {
    statusElement.textContent = message;
    console.log(message);
}

// Clean up on page unload
window.addEventListener('beforeunload', function() {
    if (pc) {
        pc.close();
        pc = null;
    }
}); 
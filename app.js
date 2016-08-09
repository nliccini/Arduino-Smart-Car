const btSerial = new (require('bluetooth-serial-port')).BluetoothSerialPort();
const _ = require('lodash');
const async = require('async');

const motions = [
    'f',
    'r',
    'b',
    'l'
];

btSerial.on('found', (address, name) => {
    console.log(`I Found a Device: ${address} - ${name}`); 

    btSerial.findSerialPortChannel(address, (channel) => {
	if (address !== '20-16-06-30-69-09') {
	    return;
	}
	console.log(`For address: ${address}, trying this channel: ${channel}`);

        btSerial.connect(address, channel, () => {
            console.log(`connected to address: ${address} on channel: ${channel}`);

	    const range = _.range(0, 10);

	    async.eachSeries(range, (i, callback) => {
		const motion = motions[i % motions.length];
                btSerial.write(new Buffer(motion, 'utf-8'), (err, bytesWritten) => {
                    if (err) {
		        console.log(err);
		    } else {
			console.log(`Successfully Wrote: ${bytesWritten} Bytes over Bluetooth`);
		    }

		    setTimeout(() => {
		        callback(err);
		    }, 2000);

                });
            }, (err) => {
		if (err) {
		    console.log(`Error Transmitting Data: ${err}`);
		} else {
		    console.log(`Transmission Complete`);
		}

		console.log('Sending Stop Command...');

                btSerial.write(new Buffer('s', 'utf-8'), (err, bytesWritten) => {
		    btSerial.close();
		    console.log('Done!');
		});

            });

            btSerial.on('data', (buffer) => {
                console.log(buffer.toString('utf-8'));
            });

        }, () => {
            console.log('cannot connect');
        });

    }, () => {
        console.log('found nothing');
    });
});

btSerial.inquire();

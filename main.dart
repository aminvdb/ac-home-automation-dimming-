import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:permission_handler/permission_handler.dart';
import 'dart:typed_data';
import 'dart:async';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'PWM Controller',
      theme: ThemeData(primarySwatch: Colors.blue, useMaterial3: true),
      home: const ConnectScreen(),
      debugShowCheckedModeBanner: false,
    );
  }
}

class ConnectScreen extends StatefulWidget {
  const ConnectScreen({super.key});

  @override
  State<ConnectScreen> createState() => _ConnectScreenState();
}

class _ConnectScreenState extends State<ConnectScreen> {
  bool _connecting = false;
  String _statusMessage = "";

  Future<void> connectToHC05() async {
    final hc05 = BluetoothDevice(name: "HC-05", address: "20:25:05:00:C9:AA");

    // Vraag Bluetooth-permissies
    Map<Permission, PermissionStatus> statuses = await [
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.location,
    ].request();

    if (statuses.values.any((status) => !status.isGranted)) {
      setState(() {
        _statusMessage = "❌ Toestemming geweigerd voor Bluetooth.";
      });
      return;
    }

    setState(() {
      _connecting = true;
      _statusMessage = "🔄 Verbinden met HC-05...";
    });

    try {
      final connection = await Future.any([
        BluetoothConnection.toAddress(hc05.address),
        Future.delayed(
          const Duration(seconds: 20),
          () => throw TimeoutException("⏱️ Timeout"),
        ),
      ]);

      setState(() {
        _statusMessage = "✅ Verbonden met HC-05!";
      });

      Navigator.pushReplacement(
        context,
        MaterialPageRoute(
          builder: (context) => PWMControlScreen(connection: connection),
        ),
      );
    } catch (e) {
      setState(() {
        _statusMessage =
            "❌ Verbinden mislukt: ${e is TimeoutException ? 'Timeout' : e.toString()}";
        _connecting = false;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("Bluetooth PWM Controller")),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            ElevatedButton.icon(
              icon: const Icon(Icons.bluetooth),
              label: const Text("Verbind met HC-05"),
              onPressed: _connecting ? null : connectToHC05,
              style: ElevatedButton.styleFrom(
                padding: const EdgeInsets.symmetric(
                  horizontal: 32,
                  vertical: 16,
                ),
              ),
            ),
            const SizedBox(height: 20),
            if (_statusMessage.isNotEmpty)
              Text(_statusMessage, style: const TextStyle(fontSize: 16)),
          ],
        ),
      ),
    );
  }
}

class PWMControlScreen extends StatefulWidget {
  final BluetoothConnection connection;

  const PWMControlScreen({super.key, required this.connection});

  @override
  _PWMControlScreenState createState() => _PWMControlScreenState();
}

class _PWMControlScreenState extends State<PWMControlScreen> {
  Map<int, double> pwmValues = {}; // PWM-waarden per slave
  Map<int, bool> slaveActive = {}; // Status per slave
  late StreamSubscription<Uint8List> _uartSubscription;

  @override
  void initState() {
    super.initState();

    // Luister naar inkomende UART-data van STM32
    _uartSubscription = widget.connection.input!.listen(
      (data) {
        String msg = String.fromCharCodes(data).trim();
        // Verwacht format: bijv. "1,0" = slave1 actief, slave2 inactief
        List<String> statuses = msg.split(',');
        setState(() {
          for (int i = 0; i < statuses.length; i++) {
            slaveActive[i] = statuses[i] == '1';
            if (!pwmValues.containsKey(i)) pwmValues[i] = 1.0;
          }
        });
      },
      onDone: () {
        print("UART stream gesloten");
      },
    );
  }

  void sendPWMValue(int slaveId, double value) {
    if (widget.connection.isConnected && slaveActive[slaveId] == true) {
      // Format: "slaveId:value\n"
      String command = "$slaveId:${value.toInt()}\n";
      widget.connection.output.add(Uint8List.fromList(command.codeUnits));
      print('📤 PWM waarde verzonden: $command');
    }
  }

  @override
  void dispose() {
    _uartSubscription.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('PWM Controller')),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          children: [
            Text(
              'Actieve slaves: ${slaveActive.values.where((v) => v).length}',
              style: const TextStyle(fontSize: 18),
            ),
            const SizedBox(height: 20),
            Expanded(
              child: ListView.builder(
                itemCount: slaveActive.length,
                itemBuilder: (context, index) {
                  if (slaveActive[index] != true) {
                    return const SizedBox.shrink();
                  }

                  return Card(
                    margin: const EdgeInsets.symmetric(vertical: 8),
                    child: Padding(
                      padding: const EdgeInsets.all(16),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            'Slave ${index + 1} PWM: ${pwmValues[index]?.toInt() ?? 0}',
                          ),
                          Slider(
                            value: pwmValues[index] ?? 1,
                            min: 1,
                            max: 255,
                            divisions: 255,
                            label: (pwmValues[index]?.toInt() ?? 1).toString(),
                            onChanged: (value) {
                              setState(() {
                                pwmValues[index] = value;
                              });
                            },
                          ),
                          ElevatedButton(
                            onPressed: () =>
                                sendPWMValue(index, pwmValues[index]!),
                            child: const Text('Verstuur PWM'),
                          ),
                        ],
                      ),
                    ),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}

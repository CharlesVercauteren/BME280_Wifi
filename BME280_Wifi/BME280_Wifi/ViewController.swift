//
//  ViewController.swift
//
//  BME280_Wifi
//
//  ©2021 Charles Vercauteren
//  01 mars 2021
//

import Cocoa
import Network

struct LogEntry {
    var time = ""
    var temperature = ""
    var pressure = ""
    var humidity = ""
}

// Commands for Arduino DS18B20_Wifi
let MESSAGE_EMPTY = ""
let GET_TEMPERATURE = "10"
let GET_HUMIDITY = "11"
let GET_PRESSURE = "12"
let SET_TIME = "20"
let GET_TIME = "21"
let SET_LOG_INTERVAL = "22"
let GET_LOG_INTERVAL = "23"
let GET_LOG = "30"
let GET_HOSTNAME = "40"
let SET_HOSTNAME = "41"

let LOG_INTERVAL = "600"

let UPDATE_INTERVAL = 1             // Update view (s)

let PORTNUMBER: UInt16 = 2000       //  UDP port number server

// Variable used to return message from NWConnection.receiveMessage closure
var reply = ""

class ViewController: NSViewController {

    @IBOutlet weak var hostNameFromArduinoTxt: NSTextField!
    @IBOutlet weak var hostNameTxt: NSTextField!
    @IBOutlet weak var ipAddressArduinoTxt: NSTextField!
    @IBOutlet weak var temperatureTxt: NSTextField!
    @IBOutlet weak var pressureTxt: NSTextField!
    @IBOutlet weak var humidityTxt: NSTextField!
    @IBOutlet weak var timeFromArduinoTxt: NSTextField!
    @IBOutlet weak var logIntervalFromArduinoTxt: NSTextField!
    @IBOutlet weak var logIntervalTxt: NSTextField!
    @IBOutlet weak var infoTxt: NSTextField!
    
    @IBOutlet weak var connectBtn: NSButton!
    @IBOutlet weak var saveLogBtn: NSButton!
    @IBOutlet weak var getLogBtn: NSButton!
    @IBOutlet weak var setLogIntervalBtn: NSButton!
    @IBOutlet weak var setHostNameBtn: NSButton!
    @IBOutlet weak var setTimeOnArduinoBtn: NSButton!
    
    @IBOutlet weak var logTable: NSTableView!
    
    //Update interval properties
    var timer = Timer()
    let interval = TimeInterval(UPDATE_INTERVAL)     //Seconds
    var logIntervalString = LOG_INTERVAL
    
    // First command to send to Arduino
    var commandToSend = GET_HUMIDITY
    
    //Arduino UDP server properties
    //IP via interface
    let portNumber: UInt16 = PORTNUMBER
    var server: NWConnection?

    var log = [LogEntry]()

    override func viewDidLoad() {
        super.viewDidLoad()
                
        // Enable/disable buttons
        saveLogBtn.isEnabled = false
        getLogBtn.isEnabled = false
        setLogIntervalBtn.isEnabled = false
        setTimeOnArduinoBtn.isEnabled = false
        //setHostNameBtn.isEnabled = false
        
        // Init text
        logIntervalTxt.stringValue = logIntervalString
        
        // Info for user
        infoTxt.stringValue = "Please connect to thermometer."
        
        // Init table with log
        logTable.dataSource = self
        logTable.delegate = self
    }
    
    @IBAction func intervalSelection(_ sender: NSButton) {
        switch sender.title {
        case "10 min":
            logIntervalString = "600"
        case "15 min":
            logIntervalString = "900"
        case "30 min":
            logIntervalString = "1800"
        case "60 min":
            logIntervalString = "3600"
        default:
            print("default")
        }
    }
    
    @IBAction func connectBtn(_ sender: Any) {
        // Disconnect current connection
        timer.invalidate()
        server?.forceCancel()
        
        // Update display now we are disconnected
        infoTxt.stringValue = "Connecting."
        hostNameFromArduinoTxt.stringValue = "------"
        temperatureTxt.stringValue = "--.-- °C"
        timeFromArduinoTxt.stringValue = "Time: --:--:--"
        logIntervalFromArduinoTxt.stringValue = "Log interval: -- s"

        //Create host
        let host = NWEndpoint.Host(ipAddressArduinoTxt.stringValue)
        //Create port
        let port = NWEndpoint.Port(rawValue: portNumber)!
        //Create endpoint
        server = NWConnection(host: host, port: port, using: NWParameters.udp)
        // The update handler will start questioning the Arduino
        server?.stateUpdateHandler = {(newState) in self.stateUpdateHandler(newState: newState) }
        server?.start(queue: .main)
    }
    
    private func stateUpdateHandler(newState: NWConnection.State){
        switch (newState){
        case .setup:
            print("State: Setup.")
        case .waiting:
            print("State: Waiting.")
        case .ready:
            // Connection available, start questioning the Arduino
            print("State: Ready.")
            startTimer()
            saveLogBtn.isEnabled = true
            getLogBtn.isEnabled = true
            setLogIntervalBtn.isEnabled = true
            setTimeOnArduinoBtn.isEnabled = true
            //setHostNameBtn.isEnabled = true
            
            commandToSend = GET_HOSTNAME
            sendCommand(server: server!, command: commandToSend)
            receiveReply(server: server!)
            
            commandToSend = GET_TIME
            sendCommand(server: server!, command: commandToSend)
            receiveReply(server: server!)
            
            commandToSend = GET_LOG_INTERVAL
            sendCommand(server: server!, command: commandToSend)
            receiveReply(server: server!)
            
        case .failed:
            print("State: Failed.")
        case .cancelled:
            print("State: Cancelled.")
        default:
            print("State: Unknown state.")
        }
    }
    
    private func startTimer() {
        if !timer.isValid {
            timer = Timer.scheduledTimer(timeInterval: interval,
                                         target: self,
                                         selector: #selector(timerTic),
                                         userInfo: nil,
                                         repeats: true)
        }
    }
    
    @objc func timerTic() {
        // Is there still something in receive buffer ?
        self.receiveReply(server: self.server!)
        
        // Get temperature
        switch commandToSend {
        case GET_TEMPERATURE:
            commandToSend = GET_PRESSURE
        case GET_PRESSURE:
            commandToSend = GET_HUMIDITY
        case GET_HUMIDITY:
            commandToSend = GET_TEMPERATURE
        default:
            commandToSend = GET_TEMPERATURE
        }
        self.sendCommand(server: self.server!, command: commandToSend)
        self.receiveReply(server: self.server!)
    }
    

    
    private func sendCommand(server: NWConnection, command: String) {
        server.send(content: command.data(using: String.Encoding.ascii),
                completion: .contentProcessed({error in
                     if let error = error {
                        print("error while sending data: \(error).")
                        return
                     }
                 }))
    }
    
    private func receiveReply(server: NWConnection){
        var logString = ""
        
        // Completion handler receiveMessage not called if nothing received,
        // make sure to empty reply after valid receive
        server.receiveMessage (completion: {(content, context,   isComplete, error) in
            let replyLocal = String(decoding: content ?? Data(), as:   UTF8.self)
            reply = replyLocal
        })
        
        // Remove the command from te reply (= command + " " + value)
        let firstSpace = reply.firstIndex(of: " ") ?? reply.endIndex
        let result = reply[..<firstSpace]
        
        // Evaluate the answer from the Arduino
        infoTxt.stringValue = "Connected."
        switch result {
        case GET_TEMPERATURE:
            self.temperatureTxt.stringValue = reply.suffix(from: firstSpace) + " °C"
        case GET_HUMIDITY:
            humidityTxt.stringValue = reply.suffix(from: firstSpace) + " %"
        case GET_PRESSURE:
            pressureTxt.stringValue = reply.suffix(from: firstSpace) + " hPa"
        case GET_TIME, SET_TIME:
            self.timeFromArduinoTxt.stringValue = "Time: " + String(reply.suffix(from: firstSpace))
        case GET_LOG:
            let startOfString = reply.index(after: firstSpace)  //Remove space
            logString = String(reply.suffix(from: startOfString))
            log = decodeLog(log: logString)
            print(log)
            logTable.reloadData()

        case GET_LOG_INTERVAL, SET_LOG_INTERVAL:
            logIntervalFromArduinoTxt.stringValue = "Log interval: " + String(reply.suffix(from: firstSpace))
            logTable.reloadData()

        case GET_HOSTNAME, SET_HOSTNAME:
            hostNameFromArduinoTxt.stringValue = String(reply.suffix(from: firstSpace))
        case MESSAGE_EMPTY:
            infoTxt.stringValue = "Waiting for sensor."
        default:
            print("Unknown command.")
        }
        reply = ""

    }
    
    @IBAction func setTimeOnArduinoBtn(_ sender: NSButton) {
        let date = Date()
        let calender = Calendar.current
        let hour = calender.component(.hour, from: date)
        let minute = calender.component(.minute, from: date)
        
        //Stop timer so we can set the time
        timer.invalidate()
        
        // Send SET_TIME command
        let setTimeCommand = SET_TIME + String(format: " %02d:%02d",hour,minute)
        sendCommand(server: server!, command: setTimeCommand)

        // Wait for response
        timeFromArduinoTxt.stringValue = "Time: --:--:--"
        // Receive response
        receiveReply(server: server!)
        
        // Continue
        startTimer()
    }
    
    @IBAction func viewLogBtn(_ sender: NSButton) {
        
        //Stop timer so we can get the log
        timer.invalidate()
        
        // Send GET_LOG command
        let setLogCommand = GET_LOG
        sendCommand(server: server!, command: setLogCommand)
        receiveReply(server: server!)
        
        // Continue
        startTimer()
    }
    
    @IBAction func setLogIntervalBtn(_ sender: Any) {
        //Stop timer so we can set the log interval
        timer.invalidate()
        
        // Waiting for response
        logIntervalFromArduinoTxt.stringValue = "Log interval: ---- s"
        
        // Send SET_LOG_INTERVAL command
        let setLogIntervalCommand = SET_LOG_INTERVAL + " " + logIntervalTxt.stringValue
        sendCommand(server: server!, command: setLogIntervalCommand)
        receiveReply(server: server!)
        
        // Continue
        startTimer()
    }
    
    @IBAction func setHostnameOnArduino(_ sender: Any) {
        //Stop timer so we can set the hostname
        timer.invalidate()
        
        // Wait for response
        hostNameFromArduinoTxt.stringValue = "------"
        
        // Send SET_HOSTNAME command
        let setHostnameCommand = SET_HOSTNAME +  " " + hostNameTxt.stringValue
        sendCommand(server: server!, command: setHostnameCommand)
        receiveReply(server: self.server!)
        
        // Continue
        startTimer()
    }
    
    @IBAction func saveLog(_ sender: NSButton) {
        
        var fileURL: URL? //(fileURLWithPath: "")

        // Save log to ...
        let savePanel = NSSavePanel()
        
        savePanel.title = "Save log."
        savePanel.nameFieldStringValue = "temperature.log"
        
        switch savePanel.runModal() {
        case .OK:
            fileURL = savePanel.url!
        case .cancel:
            fileURL = nil
        default:
            print("NSOpenPanel: error")
            exit(0)
        }
          
        if (fileURL != nil) {
            var str = ""
            for row in 0..<log.count {
                str += log[row].time + "\t" + log[row].temperature + "\n"
            }
            do {
                try str.data(using: .ascii)?.write(to: fileURL!)
            }
            catch {
                print("Error writing file")
            }
        }
    }
    
    @IBAction func exitBtn(_ sender: Any) {
        NSApplication.shared.terminate(self)
    }
    
    /*
    private func decodeLog(log: String) -> [LogEntry] {
        var decoded = [LogEntry]()
        var toDecode = log
        var index = log.startIndex

        while index != toDecode.endIndex {
            index = toDecode.firstIndex(of: "\n") ?? toDecode.endIndex
            if index != toDecode.endIndex {
                let entry = decodeLogEntry(entry: String(toDecode[...index]))
                decoded.append(entry)
                toDecode = String(toDecode[toDecode.index(after: index)...])
            }
        }
        
        return decoded.reversed()
    }
    
    private func decodeLogEntry(entry: String) -> LogEntry {
        var remains = ""
        var temperature = ""
        var index = entry.firstIndex(of: "\t") ?? entry.endIndex
        var time = String(entry[..<index])
        if time == "99:99:99" { time = "--:--:--" }
        if index != entry.endIndex {
            index = entry.index(after: index)
            remains = String(entry[index...])
            index = remains.firstIndex(of: "\n") ?? entry.endIndex
            temperature = String(remains[..<index])
            if time ==  "--:--:--" { temperature = "--.--" }

        }
        else {
            temperature = "--.-"
        }
        return LogEntry(time: time, temperature: temperature)
    }*/
    
    private func decodeLog(log: String) -> [LogEntry] {
        var decoded = [LogEntry]()
        var toDecode = log
        var index = log.startIndex
        
        print("---decodeLog")
        print(log)
        while index != toDecode.endIndex {
            index = toDecode.firstIndex(of: "\n") ?? toDecode.endIndex
            if index != toDecode.endIndex {
                print(String(toDecode[...index]))
                let entry = decodeLogEntry(entry: String(toDecode[...index]))
                decoded.append(entry)
                toDecode = String(toDecode[toDecode.index(after: index)...])
            }

        }
        
        print("+++decodeLog")
        return decoded.reversed()
    }
    
    private func decodeLogEntry(entry: String) -> LogEntry {
        var remains = ""
        var temperature = ""
        var pressure = ""
        var humidity = ""
        print(entry)
        var index = entry.firstIndex(of: "\t") ?? entry.endIndex
        let time = String(entry[..<index])
        if index != entry.endIndex {
            index = entry.index(after: index)
            remains = String(entry[index...])
            index = remains.firstIndex(of: "\t") ?? entry.endIndex
            temperature = String(remains[..<index])
            index = remains.index(after: index)
            remains = String(remains[index...])
            index = remains.firstIndex(of: "\t") ?? entry.endIndex
            pressure = String(remains[..<index])
            index = remains.index(after: index)
            humidity = String(remains[index...])
        }
        else {
            temperature = "--.-"
            pressure = "----.-"
            humidity = "--.-"
        }
        return LogEntry(time: time, temperature: temperature, pressure: pressure, humidity: humidity)
    }
}

extension ViewController: NSTableViewDataSource {
    func numberOfRows(in tableView: NSTableView) -> Int {
        return log.count
    }
}


extension ViewController: NSTableViewDelegate {
    func tableView(_ tableView: NSTableView, viewFor tableColumn: NSTableColumn?, row: Int) -> NSView? {
        
        var cellIdentifier: String = ""
        var text: String = ""
                
        if tableColumn == tableView.tableColumns[0] {
            text = log[row].time
            cellIdentifier = "Time"
        }
        
        if tableColumn == tableView.tableColumns[1] {
            text = log[row].temperature
            cellIdentifier = "Temperature"
        }
        
        if tableColumn == tableView.tableColumns[2] {
            text = log[row].pressure
            cellIdentifier = "Pressure"
        }
        
        if tableColumn == tableView.tableColumns[3] {
            text = log[row].humidity
            cellIdentifier = "Humidity"
        }
        
        if let cell = tableView.makeView(withIdentifier: NSUserInterfaceItemIdentifier(cellIdentifier), owner: nil)
            as? NSTableCellView {
            cell.textField?.stringValue = text
            return cell
        }
        return nil
    }
}


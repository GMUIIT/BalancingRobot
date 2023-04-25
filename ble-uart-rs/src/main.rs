use btleplug::api::{BDAddr, Central, Manager as _, Peripheral, ScanFilter, WriteType};
use btleplug::platform::Manager;
use std::error::Error;
use std::future::Future;
use std::io::{self, stdout, Write};
use std::net::TcpListener;
use std::sync::Arc;
use std::time::Duration;
use futures::StreamExt;
use uuid::Uuid;
use tokio::{task};
use stick::{Listener, Controller, Event, Remap};

const UART_SERVICE_UUID: Uuid = Uuid::from_u128(0x6E400001_B5A3_F393_E0A9_E50E24DCCA9E);
const RX_CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x6E400003_B5A3_F393_E0A9_E50E24DCCA9E);
const TX_CHARACTERISTIC_UUID: Uuid = Uuid::from_u128(0x6E400002_B5A3_F393_E0A9_E50E24DCCA9E);

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // Part 1: connect to gamepad
    println!("Waiting for controller...");
    let listener = Listener::new(Remap::new());
    let controller = listener.await;
    println!("Controller connected");

    // Part 2: connect to UART
    // Hardcoded C4 cat address
    let peripheral_address: BDAddr = BDAddr::from([0xB8, 0xD6, 0x1A, 0x6A, 0x37, 0xDA]);

    let manager = Manager::new().await?;
    let adapter_list = manager.adapters().await?;
    if adapter_list.is_empty() {
        eprintln!("No Bluetooth adapters found");
        return Ok(());
    }

    let adapter = &adapter_list[0]; // Assuming only one adapter is available
    let mut peripheral: Option<btleplug::platform::Peripheral> = None;

    for _ in 1..10 {
        adapter.start_scan(ScanFilter::default()).await?;
        tokio::time::sleep(Duration::from_secs(2)).await; // Wait for scan to complete

        let mut peripherals: Vec<btleplug::platform::Peripheral> = adapter.peripherals().await?;
        if peripherals.is_empty() {
            eprintln!("No BLE peripherals found");
            return Ok(());
        }

        println!("Scan found the following peripherals: ");
        for p in &peripherals {
            println!(" - {}", p);
        }

        let pos = peripherals
            .iter()
            .position(|x| x.address() == peripheral_address);

        match pos {
            None => { continue }
            Some(pos) => {
                peripheral = Some(peripherals.remove(pos));
                break
            }
        }
    }

    let peripheral = peripheral.ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "Timed out trying to find peripheral"))?;
    let peripheral = Arc::from(peripheral);

    peripheral.connect().await?;

    let properties = peripheral
        .properties()
        .await?
        .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "Properties not found"))?;

    let local_name = properties
        .local_name
        .unwrap_or_else(|| String::from("(peripheral name unknown)"));
    println!("Connecting to peripheral: {:?}", local_name);
    peripheral.connect().await?;
    println!("Connected to peripheral: {:?}", local_name);

    peripheral.discover_services().await?;

    // Get TX and RX
    let characteristics = peripheral.characteristics();
    let tx_characteristic = characteristics
        .iter()
        .find(|x| x.service_uuid == UART_SERVICE_UUID && x.uuid == TX_CHARACTERISTIC_UUID)
        .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "TX characteristic not found"))?;
    let rx_characteristic = characteristics
        .iter()
        .find(|x| x.service_uuid == UART_SERVICE_UUID && x.uuid == RX_CHARACTERISTIC_UUID)
        .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "RX characteristic not found"))?;

    // Setup RX event listener
    peripheral.subscribe(rx_characteristic).await?;

    // Task to handle incoming RX data
    let p2 = peripheral.clone();
    task::spawn(async move {
        loop {
            // Wait for next notification
            let notif = p2
                .notifications()
                .await.unwrap()
                .take(1)
                .next()
                .await
                .ok_or_else(|| io::Error::new(io::ErrorKind::BrokenPipe, "RX failed")).unwrap();
            stdout().write(notif.value.as_slice()).unwrap();
        }
    });

    // Task to handle incoming gamepad events
    let p3 = peripheral.clone();
    // task::spawn(async move {
    //     loop {
    //         let evt = controller.
    //     }
    // });

    println!("Connected to emulated UART on peripheral: {:?}", local_name);
    println!("Enter commands (Ctrl+C to exit):");

    loop {
        print!("> ");
        stdout().flush()?;
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        if input.is_empty() {
            continue;
        }
        peripheral.write(tx_characteristic, input.as_bytes(), WriteType::WithoutResponse).await?;
    }
}

import logging
from datetime import datetime
from dataclasses import dataclass
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np
import pandas as pd
import time

# drivers
from drivers.alicat import Alicat
from drivers.alicat_serial import AlicatSerial
from drivers.rigol_dp2031 import RigolDP2301
from lakeshore import Model336

logging.getLogger('lakeshore').setLevel(logging.WARNING)
logging.basicConfig(level=logging.INFO)

QTM_COLORS = ['#E1F6F2', '#A5E5D7', '#69D3BE', '#30A08E', '#1D605B',
              '#69D3BE', '#8064A2', '#E75D72', '#FF9A56']


@dataclass
class TestConfig:
    alicat_ip: str
    power_supply_ip: str
    lakeshore_ip: str
    alicat_type: str 
    alicat_com: str 
    rigol_channel: int = 1
    rigol_current_limit: float = 1.5
    coldhead_channel: int = 2
    test_name : str


class LiquidLineTest:
    def __init__(self, config: TestConfig, data_dir: str = "./data"):
        self.config = config
        self.data_dir = data_dir
        self.timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        self.logger = logging.getLogger(__name__)

        self.alicat = None
        self.rigol = None
        self.lakeshore = None
        self.df = None

    def connect(self):
        cfg = self.config

        try:
            if cfg.alicat_type == "modbus":
                self.alicat = Alicat(name='LiquidLine', host=cfg.alicat_ip, logger=self.logger)
            else:
                self.alicat = AlicatSerial("LiquidLine", cfg.alicat_com, logger=self.logger)
            self.alicat.initialize()

            self.rigol = RigolDP2301(resource_string=f"TCPIP0::{cfg.power_supply_ip}::INSTR")
            self.rigol.connect()
            self.rigol.set_voltage(cfg.rigol_channel, 0.0)
            self.rigol.set_current_limit(cfg.rigol_channel, cfg.rigol_current_limit)

            self.lakeshore = Model336(ip_address=cfg.lakeshore_ip, timeout=5.0)
        except Exception as e:
            self.logger.error(f"Error during initalization: {e}")
            self.disconnect()
            raise

    def disconnect(self):
        cfg = self.config
        if self.rigol:
            self.rigol.set_voltage(cfg.rigol_channel, 0.0)
            self.rigol.enable_output(cfg.rigol_channel, False)
            self.rigol.disconnect()
        if self.alicat:
            self.alicat.close()

    def run(self, flow_rates, voltages, flow_settle_time=300, voltage_settle_time=120):
        cfg = self.config
        data = []

        try:
            self.rigol.enable_output(cfg.rigol_channel, True)

            for flow in flow_rates:
                self.logger.info(f"Setting flow to: {flow}")
                self.alicat.set_flow_rate(flow)
                time.sleep(flow_settle_time)

                for voltage in voltages:
                    self.logger.info(f"Setting voltage to: {voltage}")
                    self.rigol.set_voltage(cfg.rigol_channel, voltage)
                    time.sleep(voltage_settle_time)

                    _, current, power = self.rigol.read_measurements(cfg.rigol_channel)
                    temps = self.lakeshore.get_all_kelvin_reading()

                    data.append({
                        'flow_rate': flow,
                        'voltage': voltage,
                        'current': current,
                        'power': power,
                        'temperature': temps[cfg.coldhead_channel]
                    })

                    self.logger.info(f"Flow={flow:.1f} SLPM, {voltage:.2f}V -> {current:.3f}A, {power:.2f}W, {temps[cfg.coldhead_channel]:.2f}K")
        finally:
            self.disconnect()

        self.df = pd.DataFrame(data)
        self.df.to_csv(f"{self.data_dir}/liquid_line_test_{cfg.test_name}_{self.timestamp}.csv", index=False)
        return self.df

    def plot(self, uncertainty=0.250, show=True):
        cfg = self.config
        if self.df is None or self.df.empty:
            return

        fig, ax = plt.subplots(figsize=(10, 6))

        for i, (flow_rate, group) in enumerate(self.df.groupby('flow_rate')):
            color = QTM_COLORS[i % len(QTM_COLORS)]
            p = group['power'].values
            t = group['temperature'].values

            if len(p) > 1:
                idx = np.argsort(p)
                p, t = p[idx], t[idx]

                p_smooth = np.linspace(p.min(), p.max(), 100)
                t_smooth = interp1d(p, t, kind='linear')(p_smooth)

                ax.plot(p_smooth, t_smooth, color=color, linewidth=2, label=f'{flow_rate:.0f} SLPM')
                ax.errorbar(p, t, yerr=uncertainty, fmt='o', color=color,
                            markersize=4, capsize=4, capthick=1.5, zorder=5)

        ax.set_ylabel('Coldhead Temperature (K)', fontsize=12)
        ax.set_xlabel('Coldtip Heater Power (W) + Parasitic (W)', fontsize=12)
        ax.set_title('CET Liquid Helium Test - Coldtip Heater', fontsize=14)
        ax.legend(loc='upper left', fontsize=11)
        ax.grid(True, which='major', linestyle='-', linewidth=0.7, alpha=0.7)
        ax.grid(True, which='minor', linestyle='--', linewidth=0.4, alpha=0.5)
        ax.minorticks_on()

        fig.tight_layout()
        plt.savefig(f"{self.data_dir}/liquid_line_test_{cfg.test_name}_{self.timestamp}.png", dpi=150)

        if show:
            plt.show()
        return fig, ax


if __name__ == "__main__":
    config = TestConfig(
        alicat_type="modbus",
        alicat_ip="192.168.22.143",
        power_supply_ip="192.168.22.141",
        lakeshore_ip="192.168.22.142",
        coldhead_channel=3
    )

    test = LiquidLineTest(config)
    test.connect()
    test.run(
        flow_rates=np.arange(20, 45, 5),
        voltages=[0.0, 2.10, 2.98, 3.65, 4.21, 4.71, 5.16],
    )
    test.plot()

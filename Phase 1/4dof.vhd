library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ring_onehot_counter is
  port (
    clk  : in  std_logic;
    rst  : in  std_logic;  -- synchronous active-high reset
    step : in  std_logic;  -- advance on rising edge when step='1'
    dir  : in  std_logic;  -- '1' = rotate left, '0' = rotate right
    q    : out std_logic_vector(3 downto 0)  -- one-hot outputs
  );
end entity ring_onehot_counter;

architecture rtl of ring_onehot_counter is
  signal r_q : std_logic_vector(3 downto 0);
begin

  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        r_q <= "0001";  -- initial state: Joint0 active
      elsif step = '1' then
        if dir = '1' then
          -- rotate left: {q2,q1,q0,q3}
          r_q <= r_q(2 downto 0) & r_q(3);
        else
          -- rotate right: {q0,q3,q2,q1}
          r_q <= r_q(0) & r_q(3 downto 1);
        end if;
      end if;
    end if;
  end process;

  q <= r_q;

end architecture rtl;

entity tb_ring is
--  no ports
end entity tb_ring;

architecture sim of tb_ring is
  signal clk  : std_logic := '0';
  signal rst  : std_logic := '1';
  signal step : std_logic := '0';
  signal dir  : std_logic := '1';
  signal q    : std_logic_vector(3 downto 0);

  constant clk_period : time := 10 ns;

  -- Component declaration (ensures tb can instantiate the unit regardless of compile order)
  component ring_onehot_counter is
    port (
      clk  : in  std_logic;
      rst  : in  std_logic;  -- synchronous active-high reset
      step : in  std_logic;  -- advance on rising edge when step='1'
      dir  : in  std_logic;  -- '1' = rotate left, '0' = rotate right
      q    : out std_logic_vector(3 downto 0)  -- one-hot outputs
    );
  end component;

begin

  -- instantiate the unit under test using component instantiation
  uut: ring_onehot_counter
    port map (
      clk  => clk,
      rst  => rst,
      step => step,
      dir  => dir,
      q    => q
    );

  -- clock generator
  clk_proc: process
  begin
    wait for clk_period/2;
    clk <= not clk;
    if now > 1000 ns then
      wait; -- stop toggling after long simulation time
    end if;
  end process clk_proc;

  -- stimulus process
  stim_proc: process
  begin
    -- initial reset pulse (synchronous)
    rst <= '1';
    wait for 25 ns;
    rst <= '0';
    wait for 10 ns;

    -- rotate left: 8 steps
    dir <= '1';
    for i in 0 to 7 loop
      step <= '1';
      wait for clk_period; -- step asserted across rising edge
      step <= '0';
      wait for clk_period;
    end loop;

    -- rotate right: 4 steps
    dir <= '0';
    for i in 0 to 3 loop
      step <= '1';
      wait for clk_period;
      step <= '0';
      wait for clk_period;
    end loop;

    -- Hold simulation for a short while then finish
    wait for 100 ns;
    report "Simulation finished" severity note;
    wait;
  end process stim_proc;

end architecture sim;

namespace Client
{
    public interface INPCStateSwitcher
    {
        void SwitchState<T>() where T : BaseNPCState;
    }
}
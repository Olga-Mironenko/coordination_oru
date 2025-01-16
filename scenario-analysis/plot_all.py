import pickle

import pandas as pd
import plotly.io as pio
import plotly.express as px
from plotly.subplots import make_subplots
from IPython.display import display, HTML

RUNDIRS = '../logs/rundirs'

pio.templates.default = "plotly_dark"


def get_key2df(runname):
    global CACHE_KEY2DF

    try:
        CACHE_KEY2DF
    except NameError:
        CACHE_KEY2DF = {}
    # Note: "When a module is reloaded, its dictionary (containing the module’s global variables) is retained."
    # (https://docs.python.org/3/library/importlib.html#importlib.reload).

    if runname not in CACHE_KEY2DF:
        rundir = f'{RUNDIRS}/{runname}'
        with open(f'{rundir}/key2df.pickle', 'rb') as file:
            key2df = pickle.load(file)

        CACHE_KEY2DF[runname] = key2df

    return CACHE_KEY2DF[runname]


MARGIN = dict(t=100, b=0, l=80, r=0)


def calculate_heatmap_data(*, df, col_i, col_j, col_data):
    return df[[col_i, col_j, col_data]].pivot(
        index=col_i, columns=col_j, values=col_data
    )


def add_ticks(*, fig, offset_fig, col_i, col_j, heatmap_data, row2label=str, idx_x, idx_y, secondary_y=False):
    idx_x += offset_fig
    idx_y += offset_fig * 2
    # print(f'{idx_x=}, {idx_y=}')

    # Apply axis settings to each subplot
    xaxis_key = 'xaxis' + (str(idx_x + 1) if idx_x > 0 else '')
    yaxis_key = 'yaxis' + (str(idx_y + 1) if idx_y > 0 else '')

    # Update axis title based on whether it's a secondary y-axis
    yaxis_title = "no. of OPs" if secondary_y else col_i

    # Set a larger title_standoff for the secondary y-axis to move the label further from the chart
    standoff_value = 10 if secondary_y else 0

    fig.update_layout(**{
        xaxis_key: dict(
            title=col_j,
            tickmode="array",
            tickvals=list(heatmap_data.columns),
            ticktext=[f'{x}' for x in heatmap_data.columns],
            title_standoff=7,  # Move x-axis title closer
            automargin=False,
            constrain='domain',
            # margin=MARGIN,
        ),
        yaxis_key: dict(
            title=yaxis_title,
            tickmode="array",
            tickvals=list(heatmap_data.index),
            ticktext=[row2label(y) for y in heatmap_data.columns],
            autorange="reversed",  # Reverse the y-axis for top-to-bottom ticks
            title_standoff=standoff_value,  # Move y-axis title closer
            automargin=False,
            constrain='domain',
            # margin=MARGIN,
        ),
    })


def add_all_ticks(*, fig, offset_fig, col_i, col_j, heatmap_data, idx, are_bridges):
    # Dictionary to map Map IDs to the number of OPs
    map_to_ops = {
        1: 2, 6: 2, 10: 2,  # Maps with 2 OPs
        2: 1, 3: 1, 4: 1, 5: 1, 7: 1, 8: 1, 9: 1,  # Maps with 1 OP
    }

    # Check if the current map is low or high connectivity
    primary_label = 'L' if not are_bridges else 'H'  # Use 'L' for low connectivity maps

    add_ticks(
        fig=fig, offset_fig=offset_fig, col_i=col_i, col_j=col_j, heatmap_data=heatmap_data,
        idx_x=idx, idx_y=idx * 2,
        row2label=lambda r: f'{r}{primary_label}',
    )

    add_ticks(
        fig=fig, offset_fig=offset_fig, col_i=col_i, col_j=col_j, heatmap_data=heatmap_data,
        idx_x=idx, idx_y=idx * 2 + 1,
        row2label=lambda r: f'{map_to_ops[r]}',
        secondary_y=True
    )


def set_hovertemplate(*, fig, col_i, col_j, col_data, is_value_in_tooltip):
    hovertemplate = ""
    # hovertemplate += col_i + ": %{y}<br>"  # `%{y}` is interpreted as the label of the RIGHT (secondary) axis
    hovertemplate += col_j + ": %{x}<br>"
    if is_value_in_tooltip:
        hovertemplate += col_data + ": %{z}"
    hovertemplate += "<extra></extra>"

    fig.update_traces(hovertemplate=hovertemplate)


def plot_df_csd(*, title, col_i, col_j, col_data, heatmap_data):
    # Create an interactive heatmap
    fig = px.imshow(
        heatmap_data,
        labels={"x": col_j, "y": col_i, "color": col_data},
        title=title,
    )

    set_hovertemplate(fig=fig, col_i=col_i, col_j=col_j, col_data=col_data, is_value_in_tooltip=True)

    # Add the secondary y-axis for 'no_ops'
    fig.update_layout(
        yaxis2=dict(
            title="no. of OPs",
            overlaying="y",
            side="right",
            tickmode="array",
            tickvals=list(heatmap_data.index),
            ticktext=[f"{2 if y in [1, 6, 10] else 1}" for y in heatmap_data.index],
            automargin=True,
        )
    )

    return fig


def make_subplots_row(titles, *, horizontal_spacing=0.05):
    if len(titles) == 2:
        horizontal_spacing = 0.25 / len(titles)
        width = 500 * len(titles)  # Adjust based on the number of subplots
        height = 500  # Fixed height to make the plots square
    elif len(titles) == 5:
        horizontal_spacing = 0.25 / len(titles)
        width = 460 * len(titles)  # Adjust based on the number of subplots
        height = 500  # Fixed height to make the plots square
    else:
        horizontal_spacing = 0.25 / len(titles)
        width = 500 * len(titles)  # Adjust based on the number of subplots
        height = 500  # Fixed height to make the plots square

    fig = make_subplots(
        rows=1,
        cols=len(titles),
        subplot_titles=titles,
        specs=[[{'secondary_y': True}] * len(titles)],
        horizontal_spacing=horizontal_spacing,  # Adjust spacing between subplots
    )

    # Set fixed dimensions to ensure square charts
    fig.update_layout(
        autosize=False,
        width=width,
        height=height,
    )

    return fig


def plot_csd_scores(runname):
    col_i = 'i_map'
    col_j = 'position'
    col_data = 'CSD score (AVs)'

    display(HTML(f'<h1>CSD scores</h2>'))
    key2df = get_key2df(runname)

    list_are_bridges = [False, True]
    titles = ['Maps with low connectivity', 'Maps with high connectivity']

    # Create subplots
    fig = make_subplots_row(titles, horizontal_spacing=0.14)

    for idx, are_bridges in enumerate(list_are_bridges):
        df = key2df[are_bridges, 'conf']
        heatmap_data = calculate_heatmap_data(df=df, col_i=col_i, col_j=col_j, col_data=col_data)

        # Determine primary label (L for low connectivity, H for high connectivity)
        primary_label = 'L' if not are_bridges else 'H'

        heatmap_fig = plot_df_csd(
            title=f'{titles[idx]}',  # TODO: `subplot_titles`
            col_i=col_i,
            col_j=col_j,
            col_data=col_data,
            heatmap_data=heatmap_data,
        )

        # Apply the same layout settings to the individual figure
        heatmap_fig.update_layout(
            title=dict(
                # text=f'{titles[idx]}',
                x=0.57,
                y=0.85,
                xanchor='center',
                yanchor='top',
            ),
            autosize=False,
            width=500,  # Same fixed width as the subplot
            height=500,  # Same fixed height as the subplot
            coloraxis=dict(
                colorscale="Greys",
                colorbar=dict(
                    title="POD score",
                    titlefont=dict(size=16),
                    x=-0.3,
                    titleside="top",
                    thickness=10,
                    len=1.1  # ⬅ Increases the length of the colorbar (default is 0.5)
                )
            ),  # Colorbar on the left

            xaxis=dict(
                title=col_j,
                tickmode="array",
                tickvals=list(heatmap_data.columns),
                ticktext=[f'{x}' for x in heatmap_data.columns],
                constrain='domain',
                automargin=False,
                # margin=MARGIN,
            ),
            yaxis=dict(
                title=f'{col_i} ({primary_label})',
                tickmode="array",
                tickvals=list(heatmap_data.index),
                ticktext=[f'{y}{primary_label}' for y in heatmap_data.index],
                autorange="reversed",
                title_standoff=0.0,  # Move the y-axis label closer to the chart
                constrain='domain',
                automargin=False,
                # margin=MARGIN,
            ),
            yaxis2=dict(
                title='no. of OPs',
                overlaying='y',
                side='right',
                tickmode="array",
                tickvals=list(heatmap_data.index),
                ticktext=[f'{2 if y in [1, 6, 10] else 1}' for y in heatmap_data.index],
                constrain='domain',
                automargin=False,
                # margin=MARGIN,
            ),
            margin=dict(l=50, r=50, t=80, b=50)  # Adjust margins for better spacing
        )

        # Add heatmap to the subplot
        for trace in heatmap_fig.data:
            trace.update(coloraxis="coloraxis1")  # Link each subplot to the shared color axis
            trace.update(showscale=True)
            fig.add_trace(trace, row=1, col=idx + 1, secondary_y=False)
            fig.add_trace(trace, row=1, col=idx + 1, secondary_y=True)

            # Add ticks for the subplot
        add_all_ticks(fig=fig, offset_fig=0, col_i='map ID', col_j='positions configuration', heatmap_data=heatmap_data,
                      idx=idx, are_bridges=are_bridges)

        # for axis in fig.layout:
        #     if axis.startswith('xaxis') or axis.startswith('yaxis'):
        #         print(axis)

        # Show the individual figure
        heatmap_fig.show()

        # Show the individual figure separately
    #   heatmap_fig.show()

    fig.update_layout(
        coloraxis1=dict(  # shared color axis
            colorscale="Greys",
            colorbar=dict(
                title="POD score",
                titlefont=dict(size=16),
                x=-0.13,  # Colorbar position for subplots
                titleside="top",
                thickness=10,
                len=1.1  # ⬅ Increases the length of the colorbar (default is 0.5)
            )
        ),
        xaxis=dict(scaleanchor="y"),  # Ensures the x-axis and y-axis are scaled equally
        yaxis=dict(scaleanchor="x"),  # Ensures the y-axis and x-axis are scaled equally
    )

    # Display the figure
    fig.show()

    display(HTML('<hr/>'))


plot_csd_scores('20241230_173555')
COLUMN_TO_IS_THE_MORE_THE_BETTER = {
    'No. of completed missions': True,
    'No. of collisions': False,
    'No. of near-misses': False,
    'Collisions rate': False,
}
TITLE_CMP = 'Strategies Comparison'


def rank_dataframes(dataframes, is_the_more_the_better=True):
    ranked_dfs = [pd.DataFrame(index=df.index, columns=df.columns)
                  for df in dataframes]

    # Get the shape of the DataFrames
    num_rows, num_cols = dataframes[0].shape

    # Iterate over each cell in the DataFrame
    for row in range(num_rows):
        for col in range(num_cols):
            # Extract the values across all DataFrames at the same position
            cell_values = (df.iloc[row, col] for df in dataframes)
            series = pd.Series(cell_values)
            ranks = series.rank(method='min', ascending=not is_the_more_the_better)

            # Iterate over each DataFrame
            for df_index, ranked_df in enumerate(ranked_dfs):
                ranked_df.iloc[row, col] = ranks[df_index]

    return ranked_dfs


def compare_ranked_dfs(df_a, df_b, label_a, label_b, color_a, color_b, color_ab):
    df_cmp = pd.DataFrame(index=df_a.index, columns=df_a.columns)
    df_colors = pd.DataFrame(index=df_a.index, columns=df_a.columns)

    # Get the shape of the DataFrames
    num_rows, num_cols = df_a.shape

    # Iterate over each cell in the DataFrame
    for row in range(num_rows):
        for col in range(num_cols):
            # Extract the values across all DataFrames at the same position
            value_a = df_a.iloc[row, col]
            value_b = df_b.iloc[row, col]

            if value_a < value_b:
                value_c = label_a
                color = color_a
            elif value_a > value_b:
                value_c = label_b
                color = color_b
            else:
                value_c = label_a + label_b
                color = color_ab

            df_cmp.iloc[row, col] = value_c
            df_colors.iloc[row, col] = color

    return df_cmp, df_colors


def plot_df(*, runname, title, col_i, col_j, col_data, heatmap_data, df_ranks, is_value_in_tooltip):
    # Create an interactive heatmap
    fig = px.imshow(
        heatmap_data,
        labels={"x": col_j, "y": col_i, "color": col_data},
        title=f"{runname}: {title}: {col_data}",
        # text_auto=True,
    )

    # fig.data[0].update(text=df_ranks.values, texttemplate="%{text}")

    # Show ranking text only for the "Strategies Comparison" heatmap
    if df_ranks is not None:
        fig.data[0].update(text=df_ranks.values, texttemplate="%{text}")

    set_hovertemplate(fig=fig, col_i=col_i, col_j=col_j, col_data=col_data, is_value_in_tooltip=is_value_in_tooltip)

    return fig


def plot_df_all(
        runname, col_data, *,
        fig, offset_fig, strategies, titles, df, are_bridges, slowness,
        is_full, is_baseline_only, is_comparison_only
):
    assert is_full + is_baseline_only + is_comparison_only == 1

    col_strategy = 'forcing'
    col_i = 'i_map'
    col_j = 'position'
    is_the_more_the_better = COLUMN_TO_IS_THE_MORE_THE_BETTER[col_data]

    title_to_heatmap_data = {}
    for idx, strategy in enumerate(strategies):
        heatmap_data = calculate_heatmap_data(
            df=df[df[col_strategy] == strategy],
            col_i=col_i,
            col_j=col_j,
            col_data=col_data,
        )
        title_to_heatmap_data[strategy] = heatmap_data

    if is_full or is_comparison_only:
        dfs_ranks = rank_dataframes(list(title_to_heatmap_data.values()),
                                    is_the_more_the_better=is_the_more_the_better)
        df_cmp, df_color = compare_ranked_dfs(
            dfs_ranks[1], dfs_ranks[2],
            strategies[1].split()[-1][0].upper(),
            strategies[2].split()[-1][0].upper(),
            0.0,  # 'blue',
            1.0,  # 'yellow',
            0.5,  # 'gray',
        )
        title_to_heatmap_data[TITLE_CMP] = df_color

        # Modify ranks for comparison heatmap if slowness is "with rerouting"
        if slowness == 'with rerouting':
            df_cmp = df_cmp.replace({'P': 'PR', 'S': 'SR', 'PS': 'PSR'})

    for idx, title in enumerate(titles):
        heatmap_data = title_to_heatmap_data[title]

        heatmap_fig = plot_df(
            runname=runname,
            title=title,
            col_i=col_i,
            col_j=col_j,
            col_data=col_data,
            heatmap_data=heatmap_data,
            df_ranks=df_cmp if title == TITLE_CMP else None,
            is_value_in_tooltip=title != TITLE_CMP,
        )

        # Add heatmap to the subplot
        for trace in heatmap_fig.data:
            if title != TITLE_CMP:
                trace.update(coloraxis="coloraxis1")  # Link each subplot to the shared color axis
            else:
                trace.update(coloraxis="coloraxis2", showscale=False)
            trace.update(showscale=title != TITLE_CMP)  # Show colorbar only if it's not the Comparison plot
            for secondary_y in False, True:
                fig.add_trace(trace, row=1, col=idx + 1 + offset_fig, secondary_y=secondary_y)

        add_all_ticks(fig=fig, offset_fig=offset_fig, col_i='map ID', col_j='positions configuration',
                      heatmap_data=heatmap_data, idx=idx, are_bridges=are_bridges)

    # Update layout with shared color scale
    fig.update_layout(
        title="" if is_comparison_only else f"{col_data}<br>(slowness: {slowness}; coordination strategies)",
        coloraxis1=dict(  # shared color axis
            colorscale="Greens" if is_the_more_the_better else "Reds",
            colorbar=dict(
                title=(
                    "No. of<br>completed<br>missions"
                    if col_data == "No. of completed missions"
                    else
                    'No. of<br>collisions'
                    if col_data == 'No. of collisions'
                    else
                    col_data
                ),  # ⬅ Conditional title
                titlefont=dict(size=14),
                x=-0.13,
                titleside="top",
                thickness=10,
                len=1.1  # ⬅ Increases the length of the colorbar (default is 0.5)
            )
        ),
        coloraxis2=dict(
            colorscale=[
                [0.0, '#FFFFE0'],  # 0.0 is P
                [0.5, '#D3D3D3'],  # 0.5 is PS
                [1.0, '#ADD8E6'],  # 1.0 is S
            ],
        )
    )


def make_comparison_title(are_bridges, slowness):
    if slowness == 'baseline':
        comment = 'Violations by MV: priorities'
    else:
        comment = 'Violations by MV: priorities, speed'
    return f'{TITLE_CMP}<br>Maps with {"high" if are_bridges else "low"} connectivity<br>{comment}'


def plot_runname(runname, column, *, is_baseline_only=False, is_comparison_only=False):
    is_full = not (is_baseline_only or is_comparison_only)
    assert is_full + is_baseline_only + is_comparison_only == 1

    display(HTML(f'<h1>{column}</h2>'))

    if is_baseline_only:
        strategies = titles = ['baseline']
        fig = make_subplots_row(['baseline (Maps with low connectivity)', 'baseline (Maps with high connectivity)'])
        offset_fig = 0
    elif is_comparison_only:
        titles = [TITLE_CMP]
        fig = make_subplots_row([
            make_comparison_title(are_bridges, slowness)
            for slowness in ('baseline', 'without rerouting', 'with rerouting')
            for are_bridges in (False, True)
            if not (not are_bridges and slowness == 'with rerouting')
        ])
        offset_fig = 0

    for are_bridges in False, True:
        if is_full:
            label_are_bridges = 'Maps with high connectivity' if are_bridges else 'Maps with low connectivity'
            display(HTML(f'<h2>{label_are_bridges}</h2>'))

        slownesses = ['baseline']
        if is_full or is_comparison_only:
            slownesses += [
                'without rerouting',
                'with rerouting',
            ]

        key2df = get_key2df(runname)
        is_aut = 'all' if column.endswith(' rate') else True
        df = key2df[are_bridges, is_aut]

        for slowness in slownesses:
            dfx = df[~df['passhum'] & (df['slowness'] == slowness)]
            assert dfx.empty == (not are_bridges and slowness == 'with rerouting')
            if dfx.empty:
                continue

            if is_full or is_comparison_only:
                col_strategy = 'forcing'
                strategies = list(dfx[col_strategy].unique())
                if is_full:
                    titles = strategies + [TITLE_CMP]
                    fig = make_subplots_row(titles)
                    offset_fig = 0

            plot_df_all(
                runname,
                column,
                fig=fig,
                offset_fig=offset_fig,
                strategies=strategies,
                titles=titles,
                df=dfx,
                are_bridges=are_bridges,
                slowness=slowness,
                is_full=is_full,
                is_baseline_only=is_baseline_only,
                is_comparison_only=is_comparison_only,
            )
            offset_fig += len(titles)

            if is_full:
                fig.show()

    if is_baseline_only or is_comparison_only:
        fig.show()

    display(HTML('<hr/>'))


def main():
    plot_runname('20241230_173555', 'Collisions rate')
    plot_runname('20241230_173555', 'No. of completed missions', is_comparison_only=True)
    plot_runname('20241230_173555', 'No. of collisions', is_comparison_only=True)
    plot_runname('20241230_173555', 'No. of completed missions', is_baseline_only=True)
    plot_runname('20241230_173555', 'No. of completed missions')
    plot_runname('20241230_173555', 'No. of collisions')
    plot_runname('20241230_173555', 'No. of near-misses')


if __name__ == '__main__':
    main()